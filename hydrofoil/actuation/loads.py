"""
Per-surface actuator loads from a simulated trajectory.

The flight controller commands three VIRTUAL channels (delta_main, delta_roll, delta_elev); the
hardware has three PHYSICAL drives: left flap = main - roll, right flap = main + roll, elevator.
Loads, limits and saturation belong to the physical drives, so everything here is per drive.
"""

from __future__ import annotations

import numpy as np

from hydrofoil.actuation.hinge import HingeMoment, SurfaceGeometry
from hydrofoil.forces.foil import free_surface_lift_factor

DRIVES = ("left", "right", "elev")


def surface_models(p: dict) -> dict[str, HingeMoment]:
    main = SurfaceGeometry(chord=p["main_chord"], span=p["main_span"], layout=p["main_actuation"],
                           flap_frac=p["main_flap_frac"], pivot_frac=p["main_pivot_frac"],
                           j_struct=p["main_surface_inertia"])
    elev = SurfaceGeometry(chord=p["elev_chord"], span=p["elev_span"], layout=p["elev_actuation"],
                           pivot_frac=p["elev_pivot_frac"], j_struct=p["elev_surface_inertia"])
    return {"left": HingeMoment(main), "right": HingeMoment(main), "elev": HingeMoment(elev)}


def drive_angles(cmds: dict) -> dict[str, float]:
    """Physical surface angles [deg] from the controller's virtual channels."""
    return {"left": cmds["delta_main"] - cmds["delta_roll"],
            "right": cmds["delta_main"] + cmds["delta_roll"],
            "elev": cmds["delta_elev"]}


def steady_moments(vp, state, cmds, env, hinges) -> dict[str, float]:
    """Steady hydrodynamic hinge moment on each drive [N m] at one instant (+ = TE down)."""
    eta, nu = state[:6], state[6:]
    foils = {"left": vp.left_foil, "right": vp.right_foil, "elev": vp.elev_foil}
    ang = drive_angles(cmds)
    out = {}
    for name, foil in foils.items():
        nu_rel = nu.copy()
        nu_rel[:3] -= env.foil_vel_body[name]
        V, alpha = foil.flow(nu_rel)
        h_c = vp._submergence_over_chord(foil, eta, env.foil_surface_z[name])
        lf = foil.lift_slope_factor * free_surface_lift_factor(h_c)
        d = np.radians(ang[name])
        hm = hinges[name]
        if hm.g.layout == "flap":
            out[name] = hm.steady(V, alpha, d, lift_factor=lf)
        else:
            # all-moving: the deflection IS angle of attack (times the foil's effectiveness)
            cl = float(foil.tables.cl(np.degrees(alpha) + foil.flap_effectiveness * ang[name])) * lf
            out[name] = hm.steady(V, alpha, d, cl=cl)
        out[name + "_V"] = V
    return out


def output_torque(hm: HingeMoment, V, delta, m_steady, dt: float, friction: float):
    """
    Torque the drive must apply at the output shaft [N m] along a sampled trajectory.
    delta in rad (array), m_steady the steady hydrodynamic moment (array), V flow speed (array).
    Returns (tau_out, d_dot, d_ddot).
    """
    d_dot = np.gradient(delta, dt)
    d_ddot = np.gradient(d_dot, dt)
    j = hm.g.j_struct + hm.i_added
    b = np.array([hm.damping(v) for v in np.atleast_1d(V)])
    tau = j * d_ddot + b * d_dot + friction * np.sign(d_dot) - m_steady
    return tau, d_dot, d_ddot
