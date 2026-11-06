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
}


class ServoDrive:
    def __init__(self, hinge: HingeMoment, spec: DriveSpec, dt: float, stop_deg: float,
                 kp: float, kd: float, torque_cap: float, delay_s: float, n_sub: int, feedforward: bool):
        self.h, self.spec, self.dt = hinge, spec, dt
        self.stop = np.radians(stop_deg)
        self.kp, self.kd, self.cap, self.n_sub, self.ff = kp, kd, torque_cap, int(n_sub), feedforward
        self.delay_steps = int(round(delay_s / dt))
        self.reset()

    def reset(self):
        self.delta = 0.0
        self.rate = 0.0
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
