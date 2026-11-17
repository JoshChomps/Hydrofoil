"""
Closed-loop surface drive: a moteus-style position controller driving a BLDC through a gearbox
against the hydrodynamic hinge moment. Replaces the kinematic Actuator (delay -> rate limit -> lag)
when params["actuator_model"] == "servo": slew rate, lag and saturation now EMERGE from current,
voltage, inertia and the water load instead of being set by hand.

Per physical drive (left flap, right flap, elevator), state (delta, delta_dot) at the output:

    moteus position mode (gains at the output shaft):
        tau_cmd   = kp (delta_cmd - delta) - kd delta_dot + tau_ff
        tau_m     = tau_cmd / N, clipped by
                    |tau_m| <= min(Kt I_max, motor peak, tau_cap / N)          (current / firmware cap)
                    I in the torque direction s <= (k V_bus - s Ke omega_m) / R_ll  (back-EMF voltage limit)
    gearbox:   tau_out = N eta tau_m   if the motor drives the load (tau_m omega_m >= 0)
                       = N tau_m / eta otherwise (load back-drives; losses help the motor)
    surface:   J_tot dd = tau_out + M_hyd(V, alpha, delta) - B(V) d_dot - tau_f tanh(d_dot / 0.05)
               J_tot = J_struct + I_added (water) + J_rotor N^2
    mechanical stops at +-stop_deg (velocity zeroed at the stop).

Compliant drive train (any of backlash_deg > 0, gear_stiffness set, a twist stiffness given): two masses
instead of one. The gear output (rotor reflected, J_m = J_rotor N^2) and the surface root (J_s = J_struct +
I_added) are joined by a torsional spring k_g with a backlash dead zone of total width b, plus light structural
damping:
        e     = (theta_m - theta_s) - clip(theta_m - theta_s, -b/2, b/2)
        tau_t = k_g e + c_g (omega_m - omega_s)   when e != 0, else 0
        J_m dw_m = tau_out(motor) - tau_t - tau_f tanh(w_m / 0.05)
        J_s dw_s = tau_t + M_hyd(delta_eff) - B(V) w_s
The surface is driven at its root and twists along its span under the distributed hinge load: for a uniform
load the lift-weighted (span-mean) deflection is delta_eff = theta_s + M_hyd(delta_eff) / k_tw with
k_tw = 3 GJ / L (L = span carried by the drive), solved with the hinge moment linearised about theta_s. The
moteus closes its loop on its encoder: "motor" (the moteus's own, on the motor side, so backlash and gear
compliance are outside the loop), "output" (an absolute encoder at the surface root; position AND damping from
the far side of the spring, which rings: docs/20) or "dual" (position from the output encoder, damping from the
motor's velocity: collocated, the standard arrangement). `delta` / `rate` are
what that encoder reads (the LQR and the supervisor see them); update() returns delta_eff, the angle that makes
lift. With no compliance configured the single-mass model below is used unchanged.

Integrated with semi-implicit Euler at n_sub sub-steps per sim step (the moteus loop itself runs
at tens of kHz; 1 ms sub-steps resolve a ~15 Hz servo comfortably). The flow (V, alpha, lift factor)
is frozen across the sub-steps, like the wave field.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from hydrofoil.actuation.drive import DriveSpec
from hydrofoil.actuation.hinge import HingeMoment

DEFAULT_SERVO_PARAMS = {
    "kp": 250.0,          # N m/rad at the output  (~15 Hz with J_tot ~ 0.03 kg m^2)  PLACEHOLDER tuning
    "kd": 4.5,            # N m s/rad at the output (zeta ~ 0.8)
    "torque_cap": 10.0,   # N m at the output, firmware limit (moteus max_torque / servo.max_current_A), docs/17 s4
    "delay_s": 0.01,      # flight-computer loop + CAN-FD transport [s]
    "n_sub": 10,          # sub-steps per sim step
    "feedforward": False, # add -M_hyd (from the hinge model) to the torque command
    # compliant drive train (see module docstring); defaults = rigid, no backlash (legacy single-mass model)
    "backlash_deg": 0.0,        # total free play at the output [deg] (planetary: 10-20 arcmin = 0.17-0.33 deg)
    "gear_stiffness": None,     # torsional stiffness gearbox + shaft at the output [N m/rad]; None = rigid
    "gear_damping_ratio": 0.05, # structural damping of that spring
    "encoder": "motor",         # "motor" (moteus on the motor side), "output" (absolute encoder at the surface) or
                                # "dual" (output-encoder position, motor-encoder velocity for the damping term)
}


class ServoDrive:
    def __init__(self, hinge: HingeMoment, spec: DriveSpec, dt: float, stop_deg: float,
                 kp: float, kd: float, torque_cap: float, delay_s: float, n_sub: int, feedforward: bool,
                 backlash_deg: float = 0.0, gear_stiffness: float | None = None, gear_damping_ratio: float = 0.05,
                 encoder: str = "motor", twist_k: float | None = None):
        self.h, self.spec, self.dt = hinge, spec, dt
        self.stop = np.radians(stop_deg)
        self.kp, self.kd, self.cap, self.n_sub, self.ff = kp, kd, torque_cap, int(n_sub), feedforward
        self.delay_steps = int(round(delay_s / dt))
        self.backlash = np.radians(backlash_deg)
        self.k_gear = gear_stiffness
        self.zeta_gear = gear_damping_ratio
        if encoder not in ("motor", "output", "dual"):
            raise ValueError(f"encoder must be 'motor', 'output' or 'dual', not {encoder!r}")
        self.encoder = encoder
        self.k_twist = twist_k
        self.compliant = backlash_deg > 0 or gear_stiffness is not None or twist_k is not None
        self.reset()

    def reset(self):
        self.delta = 0.0
        self.rate = 0.0
        self.th_m = self.w_m = self.th_s = self.w_s = 0.0      # compliant model: gear output / surface root
        self.delta_eff = 0.0
        self._peak_spring = 0.0                                 # largest torque through the gear train so far [N m]
        self._buf = deque([0.0] * self.delay_steps, maxlen=self.delay_steps) if self.delay_steps else None
        self.telemetry = {"I": 0.0, "V": 0.0, "P": 0.0, "tau_out": 0.0, "M": 0.0, "limited": False}

    @property
    def j_total(self) -> float:
        m, g = self.spec.motor, self.spec.gear
        return self.h.g.j_struct + self.h.i_added + m.j_rotor * g.ratio ** 2

    def update(self, cmd_deg: float, flow) -> float:
        """
        Advance one sim step toward cmd_deg. flow = callable(delta_rad) -> steady hydrodynamic hinge
        moment [N m] at the current flow, plus attribute .V (flow speed). Returns delta [deg].
        """
        if self._buf is not None:
            delayed = self._buf[0]
            self._buf.append(cmd_deg)
            cmd_deg = delayed
        cmd = float(np.clip(np.radians(cmd_deg), -self.stop, self.stop))
        if self.compliant:
            return self._update_compliant(cmd, flow)

        m, c, g = self.spec.motor, self.spec.ctrl, self.spec.gear
        N, eta, kt, R = g.ratio, g.efficiency, m.kt, m.r_line_line
        v_avail = c.k_mod * c.v_bus
        tau_m_lim = min(kt * c.i_peak, m.peak_torque, self.cap / N)
        J = self.j_total
        B = self.h.damping(flow.V)
        h = self.dt / self.n_sub
        i_acc = p_acc = v_pk = tau_acc = m_acc = 0.0
        limited = False
        for _ in range(self.n_sub):
            M = flow(self.delta)
            tau_cmd = self.kp * (cmd - self.delta) - self.kd * self.rate + (-M if self.ff else 0.0)
            tau_m = tau_cmd / N
            omega = N * self.rate
            s = 1.0 if tau_m >= 0 else -1.0
            i_v = max(0.0, (v_avail - s * kt * omega) / R)          # back-EMF limited current magnitude
            lim = min(tau_m_lim, kt * i_v)
            if abs(tau_m) > lim:
                tau_m, limited = s * lim, True
            tau_out = N * eta * tau_m if tau_m * omega >= 0 else N * tau_m / eta
            acc = (tau_out + M - B * self.rate - g.friction_out * np.tanh(self.rate / 0.05)) / J
            self.rate += acc * h
            self.delta += self.rate * h
            if abs(self.delta) > self.stop:
                self.delta = float(np.sign(self.delta) * self.stop)
                self.rate = 0.0
            i_q = tau_m / kt
            i_acc += i_q * h
            p_acc += (tau_m * omega + 0.75 * R * i_q * i_q + c.p_idle) * h
            v_pk = max(v_pk, kt * abs(omega) + R * abs(i_q))
            tau_acc += tau_out * h
            m_acc += M * h
        self.telemetry = {"I": i_acc / self.dt, "V": v_pk, "P": p_acc / self.dt,
                          "tau_out": tau_acc / self.dt, "M": m_acc / self.dt, "limited": limited}
        return float(np.degrees(self.delta))

    # ---------------------------------------------------------------- compliant drive train
    def twisted(self, theta_root: float, flow) -> tuple[float, float]:
        """(delta_eff, M) for a root angle: span-mean deflection under the hinge load, and that load."""
        if self.k_twist is None:
            return theta_root, float(flow(theta_root))
        m0 = float(flow(theta_root))
        e = 1e-4
        kh = (float(flow(theta_root + e)) - float(flow(theta_root - e))) / (2 * e)     # dM/d(delta), < 0
        d_eff = theta_root + m0 / (self.k_twist - kh)        # delta_eff - theta = M(delta_eff) / k_tw, linearised
        return d_eff, float(flow(d_eff))

    def _update_compliant(self, cmd: float, flow) -> float:
        m, c, g = self.spec.motor, self.spec.ctrl, self.spec.gear
        N, eta, kt, R = g.ratio, g.efficiency, m.kt, m.r_line_line
        v_avail = c.k_mod * c.v_bus
        tau_m_lim = min(kt * c.i_peak, m.peak_torque, self.cap / N)
        J_m = max(m.j_rotor * N ** 2, 1e-6)
        J_s = self.h.g.j_struct + self.h.i_added
        B = self.h.damping(flow.V)
        rigid = self.k_gear is None
        if rigid:
            n_sub, c_g = self.n_sub, 0.0
        else:
            j_red = J_m * J_s / (J_m + J_s)
            w_n = np.sqrt(self.k_gear / j_red)
            c_g = 2.0 * self.zeta_gear * np.sqrt(self.k_gear * j_red)
            n_sub = max(self.n_sub, int(np.ceil(w_n * self.dt / 0.3)))   # keep w_n h < 0.3 (semi-implicit Euler)
        h = self.dt / n_sub
        half = 0.5 * self.backlash
        i_acc = p_acc = v_pk = tau_acc = m_acc = 0.0
        limited = False
        for _ in range(n_sub):
            d_eff, M = self.twisted(self.th_s, flow)
            # position from the encoder the loop is closed on; damping from the motor side unless "output"
            # ("dual": output-encoder position + motor velocity, the collocated damping that keeps a compliant
            # drive train from ringing when the position sensor is on the far side of the spring)
            enc = self.th_m if self.encoder == "motor" else self.th_s
            enc_rate = self.w_s if self.encoder == "output" else self.w_m
            tau_cmd = self.kp * (cmd - enc) - self.kd * enc_rate + (-M if self.ff else 0.0)
            tau_m = tau_cmd / N
            omega = N * self.w_m
            s = 1.0 if tau_m >= 0 else -1.0
            i_v = max(0.0, (v_avail - s * kt * omega) / R)
            lim = min(tau_m_lim, kt * i_v)
            if abs(tau_m) > lim:
                tau_m, limited = s * lim, True
            tau_out = N * eta * tau_m if tau_m * omega >= 0 else N * tau_m / eta
            fric = g.friction_out * np.tanh(self.w_m / 0.05)
            gap = self.th_m - self.th_s
            if rigid:
                # stiff gear with free play: rigid contact on either flank, free inside the gap
                f_m = tau_out - fric                                  # net drive on the gear side
                f_s = M - B * self.w_s                                # net load on the surface side
                a_free_m, a_free_s = f_m / J_m, f_s / J_s
                sg = float(np.sign(gap))                              # free motion would press the flanks together
                closing = (self.w_m - self.w_s) * sg > 1e-9 or (a_free_m - a_free_s) * sg >= 0
                if half == 0.0 or (abs(gap) >= half - 1e-12 and closing):
                    acc = (f_m + f_s) / (J_m + J_s)
                    w = (J_m * self.w_m + J_s * self.w_s) / (J_m + J_s) if half > 0 else self.w_s
                    self.w_m = self.w_s = w + acc * h
                    self.th_s += self.w_s * h
                    self.th_m = self.th_s + (float(np.sign(gap)) * half if half > 0 else 0.0)
                    tau_t = f_m - J_m * acc
                else:
                    self.w_m += a_free_m * h
                    self.w_s += a_free_s * h
                    self.th_m += self.w_m * h
                    self.th_s += self.w_s * h
                    g2 = self.th_m - self.th_s                        # crossed a flank: put it on the flank
                    if abs(g2) > half:
                        self.th_m = self.th_s + float(np.sign(g2)) * half
                    tau_t = 0.0
            else:
                e_sp = gap - float(np.clip(gap, -half, half))
                tau_t = self.k_gear * e_sp + c_g * (self.w_m - self.w_s) if e_sp != 0.0 else 0.0
                self.w_m += (tau_out - fric - tau_t) / J_m * h
                self.w_s += (tau_t + M - B * self.w_s) / J_s * h
                self.th_m += self.w_m * h
                self.th_s += self.w_s * h
            if abs(self.th_s) > self.stop:                      # hard stop on the surface
                self.th_s = float(np.sign(self.th_s) * self.stop)
                self.w_s = 0.0
                if rigid and abs(self.th_m - self.th_s) > half:
                    self.th_m = self.th_s + float(np.sign(self.th_m - self.th_s)) * half
                    self.w_m = 0.0
            i_q = tau_m / kt
            i_acc += i_q * h
            p_acc += (tau_m * omega + 0.75 * R * i_q * i_q + c.p_idle) * h
            v_pk = max(v_pk, kt * abs(omega) + R * abs(i_q))
            tau_acc += tau_t * h
            m_acc += M * h
        self.delta, self.rate = (self.th_m, self.w_m) if self.encoder == "motor" else (self.th_s, self.w_s)
        self._peak_spring = max(getattr(self, "_peak_spring", 0.0), abs(tau_t))
        self.delta_eff, _ = self.twisted(self.th_s, flow)
        self.telemetry = {"I": i_acc / self.dt, "V": v_pk, "P": p_acc / self.dt,
                          "tau_out": tau_acc / self.dt, "M": m_acc / self.dt, "limited": limited,
                          "twist_deg": float(np.degrees(self.delta_eff - self.th_s)),
                          "gap_deg": float(np.degrees(self.th_m - self.th_s)), "spring_pk": self._peak_spring}
        return float(np.degrees(self.delta_eff))
