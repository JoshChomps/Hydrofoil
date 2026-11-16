"""
Speed-scheduled, delay-aware LQR flight controller (docs/10 Phase 3, item 2; docs/19).

One coupled MIMO law replaces the three SISO PIDs once the boat is foilborne:

    c = delta_trim(u) - K(u) [ z - z_ref, phi - phi_ref, theta - theta_ref, v, w, p, q, r,
                               int(z - z_ref), int(phi - phi_ref), int(theta - theta_ref),
                               delta - delta_trim, delta_dot, c[k-1] - delta_trim, ..., c[k-d] - delta_trim ]

on the three virtual flap channels (delta_main, delta_elev, delta_roll [deg]). The design model
(`augmented_model`) is the linearised plant (`hydrofoil.linearize`) driven by the ACHIEVED surface angles,
plus a second-order model of the drives and a chain of d = delay / dt past commands, discretised at the
control rate; the cost also penalises the command rate. Gains and trim are computed at a few speeds and
interpolated in the surge speed u (thrust stays on the surge PID). Integral action removes the steady
error that proportional state feedback leaves under a bank/pitch reference or a trim shift.

Why the drive and the delay are in the model: the earlier design (`design`, kept for reference and tests)
assumed the commanded flap was delivered instantly. Its roll and pitch loops crossed over near the drive's
own bandwidth, and it limit-cycled as soon as the loop latency reached 30 ms.

Takeoff and low speed are outside the linear model's validity, so the PID flies them; the handover is
bumpless on all three channels (the integrators are initialised so the first LQR command equals the last
PID command, within the integrator clamps) and latches.
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import expm, solve_continuous_are, solve_discrete_are

from hydrofoil.linearize import trim, linearize, reduced

STATE_IDX = [0, 1, 2, 4, 5, 6, 7, 8]          # of the reduced [z phi theta u v w p q r]: drop u (index 3)

# Bryson-style defaults: 1 / (acceptable excursion)^2. Angles in rad, rates in rad/s, inputs in degrees.
DEFAULT_WEIGHTS = {
    "z": 0.03, "phi": np.radians(0.8), "theta": np.radians(2.0),
    "v": 0.3, "w": 0.15, "p": np.radians(8.0), "q": np.radians(8.0), "r": np.radians(15.0),
    "iz": 0.03 * 1.0, "iphi": np.radians(0.8), "itheta": np.radians(2.0) * 1.0,
    "u_main": 8.0, "u_elev": 8.0, "u_roll": 8.0,
    "du_main": 250.0, "du_elev": 250.0, "du_roll": 250.0,   # command rates [deg/s] (delay-aware design only)
}


def _q_r(w: dict):
    q = np.diag([1 / w[k] ** 2 for k in ("z", "phi", "theta", "v", "w", "p", "q", "r", "iz", "iphi", "itheta")])
    r = np.diag([1 / w["u_main"] ** 2, 1 / w["u_elev"] ** 2, 1 / w["u_roll"] ** 2])
    return q, r


def design(vp, speed: float, weights: dict | None = None):
    """LQR gain K (3 x 11) and trim flap angles (3,) at one speed."""
    w = dict(DEFAULT_WEIGHTS, **(weights or {}))
    x0, u0 = trim(vp, speed)
    A, B = linearize(vp, x0, u0)
    Ar, Br, _ = reduced(A, B)
    As, Bs = Ar[np.ix_(STATE_IDX, STATE_IDX)], Br[STATE_IDX, :]
    n = len(STATE_IDX)
    # integrators of [z, phi, theta] errors: states 0, 1, 2 of the reduced set
    Aa = np.zeros((n + 3, n + 3)); Ba = np.zeros((n + 3, 3))
    Aa[:n, :n], Ba[:n] = As, Bs
    for i in range(3):
        Aa[n + i, i] = 1.0
    Q, R = _q_r(w)
    P = solve_continuous_are(Aa, Ba, Q, R)
    K = np.linalg.solve(R, Ba.T @ P)
    return K, u0[:3].copy()


def hinge_stiffness(vp, name: str, speed: float) -> float:
    """Restoring hydrodynamic hinge stiffness -dM/d(delta) [N m/rad] of one drive's surface at level flight
    (`speed`, nominal flying depth). It grows with q c_f^2: about 4 N m/rad on the 25 kg model at 4 m/s, about
    100 N m/rad on the manned flap at 8 m/s, against the servo kp of 250, so it cannot be left out."""
    from hydrofoil.forces.foil import free_surface_lift_factor
    p = vp.params
    foil = {"left": vp.left_foil, "right": vp.right_foil, "elev": vp.elev_foil}[name]
    h = vp.hinges[name]
    depth = (p["hull_offset"] - p["flying_height"]) + foil.r_attach[2]
    lf = foil.lift_slope_factor * free_surface_lift_factor(depth / foil.geometry.chord)
    q = 0.5 * h.rho * speed * speed
    if h.g.layout == "flap":
        return -q * h._ref * lf * h.ch_delta
    return -q * h._ref * h._arm * foil._a2d * lf * foil.flap_effectiveness


def servo_model(vp, speed: float):
    """Linear model of the surface drives on the three virtual channels [main, elev, roll]:
    delta_dd = g cmd - wn2 delta - c delta_d, from the moteus position gains, the total inertia at the
    output (structure + water + reflected rotor), the hydrodynamic hinge stiffness and damping at `speed`
    (wn2 = (kp + k_hinge) / J, g = kp / J: a restoring hinge moment leaves the surface short of its command).
    The current/voltage limits are left out. Returns (wn2, c, g)."""
    wn2, c, g = [], [], []
    for name in ("left", "elev", "left"):                   # main and roll both act through the flap drives
        d = vp.drives[name]
        wn2.append((d.kp + hinge_stiffness(vp, name, speed)) / d.j_total)
        c.append((d.kd + d.h.damping(speed)) / d.j_total)
        g.append(d.kp / d.j_total)
    return np.array(wn2), np.array(c), np.array(g)


def augmented_model(vp, speed: float, dt: float, delay_steps: int):
    """Discrete design model at one speed, sampled at the control rate, with what the plain LQR left out:

        x = [ z phi theta v w p q r | int z, int phi, int theta | delta (3) | delta_dot (3) | c[k-1] ... c[k-d] (3 d) ]

    delta are the ACHIEVED surface angles (deviation from trim, deg), c the commands. The plant is driven by
    delta, the drives by the command issued `delay_steps` samples ago. Returns (Ad, Bd, trim angles)."""
    x0, u0 = trim(vp, speed)
    A, B = linearize(vp, x0, u0)
    Ar, Br, _ = reduced(A, B)
    As, Bs = Ar[np.ix_(STATE_IDX, STATE_IDX)], Br[STATE_IDX, :]
    n = len(STATE_IDX)
    wn2, c, gain = servo_model(vp, speed)
    nc = n + 9                                              # continuous part: plant, integrators, delta, delta_dot
    Ac = np.zeros((nc, nc)); Bc = np.zeros((nc, 3))
    Ac[:n, :n] = As
    Ac[:n, n + 3:n + 6] = Bs
    for i in range(3):
        Ac[n + i, i] = 1.0                                  # integrators of z, phi, theta
        Ac[n + 3 + i, n + 6 + i] = 1.0                      # d(delta)/dt = delta_dot
        Ac[n + 6 + i, n + 3 + i] = -wn2[i]
        Ac[n + 6 + i, n + 6 + i] = -c[i]
        Bc[n + 6 + i, i] = gain[i]
    M = np.zeros((nc + 3, nc + 3)); M[:nc, :nc] = Ac; M[:nc, nc:] = Bc
    E = expm(M * dt)
    Ad0, Bd0 = E[:nc, :nc], E[:nc, nc:]
    d = int(delay_steps)
    N = nc + 3 * d
    Ad = np.zeros((N, N)); Bd = np.zeros((N, 3))
    Ad[:nc, :nc] = Ad0
    if d == 0:
        Bd[:nc] = Bd0
    else:
        Ad[:nc, N - 3:] = Bd0                               # the oldest stored command reaches the drives
        Bd[nc:nc + 3] = np.eye(3)                           # the new command enters the chain
        for j in range(1, d):
            Ad[nc + 3 * j:nc + 3 * j + 3, nc + 3 * (j - 1):nc + 3 * j] = np.eye(3)
    return Ad, Bd, u0[:3].copy()


def design_delay_aware(vp, speed: float, dt: float, delay_steps: int, weights: dict | None = None):
    """Discrete LQR on `augmented_model`: K (3 x (17 + 3 d)) and the trim angles. On top of the Bryson
    weights it penalises the command RATE (c[k] - c[k-1]) / dt, which is what keeps the loop bandwidth
    below the drive and latency instead of fighting them."""
    w = dict(DEFAULT_WEIGHTS, **(weights or {}))
    d = max(int(delay_steps), 1)                            # c[k-1] is needed for the rate penalty
    Ad, Bd, T = augmented_model(vp, speed, dt, d)
    N, n = Ad.shape[0], len(STATE_IDX)
    Q11, R = _q_r(w)
    Q = np.zeros((N, N)); Q[:n + 3, :n + 3] = Q11
    Rd = np.diag([1 / (w[k] * dt) ** 2 for k in ("du_main", "du_elev", "du_roll")])
    E = np.zeros((3, N)); E[:, n + 9:n + 12] = np.eye(3)    # picks c[k-1]
    Q = Q + E.T @ Rd @ E
    S = -E.T @ Rd
    Rt = R + Rd
    P = solve_discrete_are(Ad, Bd, Q, Rt, s=S)
    K = np.linalg.solve(Rt + Bd.T @ P @ Bd, Bd.T @ P @ Ad + S.T)
    return K, T


class LQRController:
    """Scheduled, delay-aware LQR on the flap channels with integral action and a latched, bumpless PID
    handover. The gains come from `design_delay_aware`: the feedback uses the plant state, the achieved
    surface angles and rates (drive encoders) and its own last `delay_steps` commands."""

    def __init__(self, vp, dt: float, speeds=(3.0, 4.0, 5.0, 6.0, 7.0), weights: dict | None = None,
                 engage_speed: float = 3.2, engage_height_err: float = 0.15, limit_deg: float = 20.0,
                 int_limit=(0.05, 0.03, 0.03), delay_s: float = 0.01, blend_s: float = 0.5):
        self.dt, self.limit, self.blend_s = dt, limit_deg, blend_s
        self.speeds = np.asarray(speeds, float)
        self.delay_steps = max(int(round(delay_s / dt)), 1)
        ks, ts = zip(*(design_delay_aware(vp, s, dt, self.delay_steps, weights) for s in self.speeds))
        self.K, self.trim = np.asarray(ks), np.asarray(ts)       # (ns, 3, 17 + 3 d), (ns, 3)
        self.engage_speed, self.engage_err = engage_speed, engage_height_err
        self.int_limit = np.asarray(int_limit)                   # |integral| clamps [m s, rad s, rad s]
        self.reset()

    def reset(self):
        self.engaged = False
        self.since_engage = 0.0
        self.integ = np.zeros(3)
        self.last = np.zeros(3)
        self.hist = np.zeros((self.delay_steps, 3))              # commands already sent, newest first [deg]

    def _sched(self, u: float):
        u = float(np.clip(u, self.speeds[0], self.speeds[-1]))
        j = int(np.clip(np.searchsorted(self.speeds, u) - 1, 0, len(self.speeds) - 2))
        f = (u - self.speeds[j]) / (self.speeds[j + 1] - self.speeds[j])
        return (1 - f) * self.K[j] + f * self.K[j + 1], (1 - f) * self.trim[j] + f * self.trim[j + 1]

    def _push(self, cmd):
        self.hist = np.vstack([cmd, self.hist[:-1]])

    def update(self, state, z_ref: float, phi_ref: float, theta_ref: float, pid_raw: dict, act=None) -> dict:
        """Raw virtual-channel commands [deg]. `pid_raw` is what the PIDs (incl. feed-forward) would command;
        `act` = (achieved [main, elev, roll] angles [deg], their rates [deg/s]) from the drives (None: assume
        the drives sit on the last command)."""
        eta, nu = state[:6], state[6:]
        z, phi, th = eta[2], eta[3], eta[4]
        u = nu[0]
        pid_vec = np.array([pid_raw["delta_main"], pid_raw["delta_elev"], pid_raw["delta_roll"]])
        K, T = self._sched(u)
        err = np.array([z - z_ref, phi - phi_ref, th - theta_ref])
        xs = np.concatenate([err, [nu[1], nu[2], nu[3], nu[4], nu[5]]])    # z phi theta | v w p q r
        ang, rate = (self.hist[0], np.zeros(3)) if act is None else (np.asarray(act[0], float), np.asarray(act[1], float))
        tail = np.concatenate([ang - T, rate, (self.hist - T).ravel()])     # drives and the command chain
        Ki, Kt = K[:, 8:11], K[:, 11:]

        if not self.engaged:
            if u >= self.engage_speed and abs(err[0]) < self.engage_err:
                self.engaged = True
                # bumpless: choose the three integrators so the first LQR command equals the PIDs' on every
                # channel (K_i integ = base - pid); clipped, so a large mismatch still leaves a small step
                base = T - K[:, :8] @ xs - Kt @ tail
                try:
                    self.integ = np.linalg.solve(Ki, base - pid_vec)
                except np.linalg.LinAlgError:
                    self.integ = np.zeros(3)
                self.integ = np.clip(self.integ, -self.int_limit, self.int_limit)
            else:
                self.last = pid_vec
                self._push(pid_vec)
                return dict(pid_raw)

        cmd_nom = T - K[:, :8] @ xs - Ki @ self.integ - Kt @ tail
        # handover blend: the integrators cannot always absorb the PID/LQR mismatch (they are clamped), so the
        # command fades from the PID's to the LQR's over blend_s instead of stepping (a 14 deg elevator step with
        # the autotuned weights, 2026-11-15)
        self.since_engage += self.dt
        a = min(1.0, self.since_engage / self.blend_s) if self.blend_s > 0 else 1.0
        if a < 1.0:
            cmd_nom = a * cmd_nom + (1.0 - a) * pid_vec
        cmd = np.clip(cmd_nom, -self.limit, self.limit)
        # conditional integration: hold an integrator while its channel is saturated and the error pushes deeper
        for i in range(3):
            sat_hi = cmd_nom[i] > self.limit and K[i, 8 + i] * err[i] < 0
            sat_lo = cmd_nom[i] < -self.limit and K[i, 8 + i] * err[i] > 0
            if not (sat_hi or sat_lo):
                self.integ[i] = np.clip(self.integ[i] + err[i] * self.dt, -self.int_limit[i], self.int_limit[i])
        self.last = cmd
        self._push(cmd)
        return {"delta_main": float(cmd[0]), "delta_elev": float(cmd[1]), "delta_roll": float(cmd[2])}
