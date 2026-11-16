"""
Electromechanical model of one control-surface drive: moteus controller -> BLDC motor -> gearbox -> surface.

Given what the surface has to do (angle, rate, acceleration) and the hydrodynamic moment on it,
this works out what the motor and controller must supply, and checks it against their limits.

Mechanics (output side, + = trailing edge down)
    tau_out = (J_struct + J_added) * dd + B_hyd * d_dot + tau_fric * sign(d_dot) - M_hyd_steady
    (J_added and B_hyd come from hinge.py; M_hyd_steady is the steady hinge moment)

Motor side, gear ratio N, gear efficiency eta:
    omega_m = N * d_dot
    tau_m   = J_rotor * N * dd  +  tau_out / (N * eta)    when the motor drives the load (power flows out)
            = J_rotor * N * dd  +  tau_out * eta / N      when the load back-drives the motor (regeneration)
    So the reflected inertia seen at the surface is J_rotor * N^2: at 20:1 the rotor usually dominates.

Electrical (dq frame, steady-state, inductance neglected -- fine below a few hundred Hz):
    Kt = Ke = 60 / (2 pi Kv)              [N m/A, V s/rad] with Kv in rpm/V (BLDC convention, check vs measured)
    I_q      = tau_m / Kt
    V_needed = Ke * omega_m + R_ll * I_q  must stay below k_mod * V_bus (Kv is quoted against the DC bus)
    P_copper = 1.5 * R_phase * I_q^2 = 0.75 * R_ll * I_q^2
    P_bus    = tau_m * omega_m + P_copper + P_idle    (negative = regeneration into the bus)

Everything marked PLACEHOLDER below must be replaced from datasheets / bench tests.
Sourced: moteus r4.11 = 10-44 V input, 100 A peak phase current, 900 W peak @ 30 V (mjbots docs);
continuous phase current is quoted as 10-12 A without and 20-32 A with heat sinking (sources disagree).
mj5208 = Kv 330, 1.7 N m peak, 7500 rpm max (reseller listing). Resistance and rotor inertia are estimates.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class MotorSpec:
    kv_rpm_per_v: float = 330.0     # mj5208 (listing)
    r_line_line: float = 0.10       # ohm, PLACEHOLDER (typical 5208 class; measure with moteus_tool calibration)
    j_rotor: float = 7.0e-5         # kg m^2, PLACEHOLDER (~100 g outrunner bell at r ~ 27 mm)
    peak_torque: float = 1.7        # N m (listing)

    @property
    def kt(self) -> float:
        return 60.0 / (2.0 * np.pi * self.kv_rpm_per_v)


@dataclass
class ControllerSpec:
    """moteus r4.11."""
    v_bus: float = 40.0             # supply [V] (battery -> PDB -> motors, 40 V); allowed 10-44 V
    supply_i_max: float = 25.0      # A, PDB output limit for the whole rail
    i_peak: float = 100.0           # A phase, peak
    i_cont: float = 12.0            # A phase, continuous WITHOUT heat sinking (20-32 A with) -- conservative
    p_peak: float = 900.0           # W
    k_mod: float = 0.90             # usable fraction of the bus voltage (PWM dead time, margin)
    p_idle: float = 1.5             # W, board quiescent PLACEHOLDER


@dataclass
class GearSpec:
    ratio: float = 20.0
    efficiency: float = 0.85        # PLACEHOLDER (2-stage planetary ~0.85-0.9)
    rated_torque: float = 30.0      # N m at the output, PLACEHOLDER (gearbox datasheet)
    friction_out: float = 0.05      # N m Coulomb friction at the output (seals, bearings) PLACEHOLDER


@dataclass
class DriveSpec:
    motor: MotorSpec = field(default_factory=MotorSpec)
    ctrl: ControllerSpec = field(default_factory=ControllerSpec)
    gear: GearSpec = field(default_factory=GearSpec)
    shaft_diameter: float = 0.008   # m, output shaft PLACEHOLDER (for torsional stress); >= 12.8 mm for the full
                                    # 28.9 N m of the 20:1 drive into a stop or jam (docs/20 section 4)
    shaft_shear_yield: float = 140e6   # Pa, ~316 stainless (0.58 * 240 MPa)


def electrical_demand(spec: DriveSpec, d_dot, d_ddot, tau_out):
    """
    Vectorised motor/controller demand for a surface trajectory.

    d_dot, d_ddot : surface rate [rad/s] and acceleration [rad/s^2] (arrays)
    tau_out       : torque the drive must apply at the output shaft [N m], + = TE down,
                    EXCLUDING the reflected rotor inertia (that is added here)
    Returns dict of arrays: omega_m, tau_m, i_q, v_needed, p_bus, p_copper.
    """
    m, c, g = spec.motor, spec.ctrl, spec.gear
    d_dot, d_ddot, tau_out = map(np.asarray, (d_dot, d_ddot, tau_out))
    N, eta = g.ratio, g.efficiency
    omega_m = N * d_dot
    driving = tau_out * d_dot >= 0.0          # power flows motor -> load (holding counts as driving)
    tau_load_m = np.where(driving, tau_out / (N * eta), tau_out * eta / N)
    tau_m = m.j_rotor * N * d_ddot + tau_load_m
    i_q = tau_m / m.kt
    v_needed = m.kt * np.abs(omega_m) + m.r_line_line * np.abs(i_q)
    p_copper = 0.75 * m.r_line_line * i_q**2
    p_bus = tau_m * omega_m + p_copper + c.p_idle
    return {"omega_m": omega_m, "tau_m": tau_m, "i_q": i_q, "v_needed": v_needed,
            "p_bus": p_bus, "p_copper": p_copper}


def budget(spec: DriveSpec, t, d_dot, d_ddot, tau_out) -> dict:
    """Summarise a trajectory against the limits. Returns scalars + pass/fail flags."""
    e = electrical_demand(spec, d_dot, d_ddot, tau_out)
    m, c, g = spec.motor, spec.ctrl, spec.gear
    dt = np.gradient(np.asarray(t))
    i_abs = np.abs(e["i_q"])
    i_rms = float(np.sqrt(np.mean(e["i_q"] ** 2)))
    tau_out_pk = float(np.max(np.abs(tau_out)))
    shaft_stress = 16.0 * tau_out_pk / (np.pi * spec.shaft_diameter**3)
    regen_j = float(-np.sum(np.minimum(e["p_bus"], 0.0) * dt))
    out = {
        "i_peak": float(i_abs.max()),
        "i_rms": i_rms,
        "tau_m_peak": float(np.abs(e["tau_m"]).max()),
        "rpm_peak": float(np.abs(e["omega_m"]).max() * 60 / (2 * np.pi)),
        "v_peak": float(e["v_needed"].max()),
        "p_bus_peak": float(e["p_bus"].max()),
        "p_bus_mean": float(np.mean(e["p_bus"])),
        "regen_energy_j": regen_j,
        "tau_out_peak": tau_out_pk,
        "tau_out_rms": float(np.sqrt(np.mean(np.asarray(tau_out) ** 2))),
        "shaft_stress_mpa": shaft_stress / 1e6,
    }
    out["ok_current_peak"] = out["i_peak"] <= c.i_peak and out["tau_m_peak"] <= m.peak_torque
    out["ok_current_rms"] = i_rms <= c.i_cont
    out["ok_voltage"] = out["v_peak"] <= c.k_mod * c.v_bus
    out["ok_power"] = out["p_bus_peak"] <= c.p_peak
    out["ok_gearbox"] = tau_out_pk <= g.rated_torque
    out["ok_shaft"] = shaft_stress <= spec.shaft_shear_yield / 2.0   # SF 2 on yield (static)
    out["ok"] = all(v for k, v in out.items() if k.startswith("ok_"))
    return out


def limits_at_output(spec: DriveSpec, j_load: float, m_hyd: float = 0.0) -> dict:
    """
    What the drive can do at the surface (all at the output shaft):
      torque_max   continuous-free peak torque available (current- and motor-limited)
      rate_max     no-load surface rate from the voltage limit [rad/s]
      accel_max    peak acceleration against a hydrodynamic moment m_hyd [rad/s^2]
      n_match      inertia-matching gear ratio sqrt(J_load / (eta J_rotor))
    """
    m, c, g = spec.motor, spec.ctrl, spec.gear
    tau_m_max = min(m.kt * c.i_peak, m.peak_torque)
    tau_out_max = tau_m_max * g.ratio * g.efficiency
    rate_max = c.k_mod * c.v_bus / m.kt / g.ratio
    j_total = j_load + m.j_rotor * g.ratio**2 * 1.0
    accel = (tau_out_max - abs(m_hyd)) / j_total
    return {"torque_max": tau_out_max, "torque_cont": m.kt * c.i_cont * g.ratio * g.efficiency,
            "rate_max": rate_max, "accel_max": accel, "j_reflected": m.j_rotor * g.ratio**2,
            "j_total": j_total, "n_match": float(np.sqrt(j_load / (g.efficiency * m.j_rotor)))}
