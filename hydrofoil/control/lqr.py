"""
Speed-scheduled LQR flight controller (docs/10 Phase 3, item 2).

One coupled MIMO law replaces the three SISO PIDs once the boat is foilborne:

    delta = delta_trim(u) - K(u) [ z - z_ref, phi - phi_ref, theta - theta_ref, v, w, p, q, r,
                                   int(z - z_ref), int(phi - phi_ref), int(theta - theta_ref) ]

on the three virtual flap channels (delta_main, delta_elev, delta_roll [deg]). Gains and trim come from the
linearised plant (`hydrofoil.linearize`) at a few speeds and are interpolated in the surge speed u, which is
treated as a slowly varying parameter (thrust stays on the surge PID). Integral action removes the
steady error that proportional state feedback leaves under a bank/pitch reference or a trim shift.

Takeoff and low speed are outside the linear model's validity, so the PID flies them; the handover is
bumpless on the heave channel (the heave integrator is initialised so the first LQR command equals the last
PID command) and latches.
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import solve_continuous_are

from hydrofoil.linearize import trim, linearize, reduced

STATE_IDX = [0, 1, 2, 4, 5, 6, 7, 8]          # of the reduced [z phi theta u v w p q r]: drop u (index 3)

# Bryson-style defaults: 1 / (acceptable excursion)^2. Angles in rad, rates in rad/s, inputs in degrees.
DEFAULT_WEIGHTS = {
    "z": 0.03, "phi": np.radians(0.8), "theta": np.radians(2.0),
    "v": 0.3, "w": 0.15, "p": np.radians(8.0), "q": np.radians(8.0), "r": np.radians(15.0),
    "iz": 0.03 * 1.0, "iphi": np.radians(0.8), "itheta": np.radians(2.0) * 1.0,
    "u_main": 8.0, "u_elev": 8.0, "u_roll": 8.0,
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


class LQRController:
    """Scheduled LQR on the flap channels with integral action and a latched, bumpless PID handover."""

    def __init__(self, vp, dt: float, speeds=(3.0, 4.0, 5.0, 6.0, 7.0), weights: dict | None = None,
                 engage_speed: float = 3.2, engage_height_err: float = 0.15, limit_deg: float = 20.0,
                 int_limit=(0.05, 0.03, 0.03)):
        self.dt, self.limit = dt, limit_deg
        self.speeds = np.asarray(speeds, float)
        ks, ts = zip(*(design(vp, s, weights) for s in self.speeds))
        self.K, self.trim = np.asarray(ks), np.asarray(ts)       # (ns, 3, 11), (ns, 3)
        self.engage_speed, self.engage_err = engage_speed, engage_height_err
        self.int_limit = np.asarray(int_limit)                   # |integral| clamps [m s, rad s, rad s]
        self.reset()

    def reset(self):
        self.engaged = False
        self.integ = np.zeros(3)
        self.last = np.zeros(3)

    def _sched(self, u: float):
        u = float(np.clip(u, self.speeds[0], self.speeds[-1]))
        K = np.empty(self.K.shape[1:]); T = np.empty(3)
        for i in range(3):
            T[i] = np.interp(u, self.speeds, self.trim[:, i])
            for j in range(self.K.shape[2]):
                K[i, j] = np.interp(u, self.speeds, self.K[:, i, j])
        return K, T

    def update(self, state, z_ref: float, phi_ref: float, theta_ref: float, pid_raw: dict) -> dict:
        """Raw virtual-channel commands [deg]. `pid_raw` is what the PIDs (incl. feed-forward) would command."""
        eta, nu = state[:6], state[6:]
        z, phi, th = eta[2], eta[3], eta[4]
        u = nu[0]
        pid_vec = np.array([pid_raw["delta_main"], pid_raw["delta_elev"], pid_raw["delta_roll"]])
        K, T = self._sched(u)
        err = np.array([z - z_ref, phi - phi_ref, th - theta_ref])
        xs = np.concatenate([err, [nu[1], nu[2], nu[3], nu[4], nu[5]]])    # z phi theta | v w p q r

        if not self.engaged:
            if u >= self.engage_speed and abs(err[0]) < self.engage_err:
                self.engaged = True
                # bumpless: choose the heave integrator so the first heave-channel command equals the PID's
                ki = K[0, 8]
                base = T[0] - K[0, :8] @ xs
                self.integ[:] = 0.0
                if abs(ki) > 1e-9:
                    self.integ[0] = np.clip((base - pid_vec[0]) / ki, -self.int_limit[0], self.int_limit[0])
            else:
                self.last = pid_vec
                return dict(pid_raw)

        cmd_nom = T - K @ np.concatenate([xs, self.integ])
        cmd = np.clip(cmd_nom, -self.limit, self.limit)
        # conditional integration: hold an integrator while its channel is saturated and the error pushes deeper
        for i in range(3):
            sat_hi = cmd_nom[i] > self.limit and K[i, 8 + i] * err[i] < 0
            sat_lo = cmd_nom[i] < -self.limit and K[i, 8 + i] * err[i] > 0
            if not (sat_hi or sat_lo):
                self.integ[i] = np.clip(self.integ[i] + err[i] * self.dt, -self.int_limit[i], self.int_limit[i])
        self.last = cmd
        return {"delta_main": float(cmd[0]), "delta_elev": float(cmd[1]), "delta_roll": float(cmd[2])}
