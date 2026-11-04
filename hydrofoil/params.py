"""Default simulation parameters, shared by the dashboard and the viewer."""

from __future__ import annotations

from typing import Any

DEFAULT_PARAMS: dict[str, Any] = {
    "mass": 25.0,
    "ixx": 2.0, "iyy": 5.0, "izz": 5.0,
    "heave_damping": 50.0,
    "main_chord": 0.15, "main_span": 0.6,
    "elev_chord": 0.10, "elev_span": 0.3,
    "main_pos_x": 0.0,  "main_pos_z": 0.5, "main_lateral_offset": 0.35,
    "elev_pos_x": -1.2, "elev_pos_z": 0.4,
    # Heave gains are tuned higher than sim.py's — here the vehicle starts at rest
    # and must generate real lift to take off, not just trim near an already-flying point.
    "kp_heave": 15.0, "ki_heave": 1.0, "kd_heave": 5.0,
    "kp_pitch": 2.0, "ki_pitch": 0.0, "kd_pitch": 2.0,
    # Ki must stay 0 here — any nonzero integral term reliably diverges under
    # wave disturbance (verified: Ki=0.05 blew up to 1000+ deg within 90s).
    "kp_roll":  3.5, "ki_roll":  0.0, "kd_roll":  2.5,
    # Target height of the HULL above the water once foilborne [m] (negative =
    # hull still below the surface). Internally converted to a CoG depth
    # reference via hull_offset — this is the number you actually care about.
    # Default = half the strut length (hull_offset + main_pos_z) / 2, so the
    # main strut's midpoint sits right at the waterline once flying — half
    # the shaft above water, half below.
    "flying_height": 0.55,
    "phi_ref": 0.0, "theta_ref": 0.0,
    "waves_enabled": False,
    "wave_amplitude": 0.05, "wave_wavelength": 5.0, "wave_period": 2.0,
    "dt": 0.01, "t_end": 60.0,

    # Velocity (surge) controller: closed-loop speed control, thrust in Newtons.
    "target_speed": 4.0,
    "kp_speed": 30.0, "ki_speed": 5.0, "kd_speed": 0.0,
    "max_thrust": 50.0,   # matches the Unity TwinMotorController placeholder (50N)

    # Hull buoyancy / drag — a simplified wall-sided hull near the waterline.
    # hull_offset is how far above the CoG the hull rides (roughly the strut
    # length); waterplane_area/max_draft set how buoyant it is at rest and how
    # much draft it has before the linear approximation caps out.
    "waterplane_area": 0.6, "max_draft": 0.3, "hull_offset": 0.6,
    "hull_drag_coeff": 20.0,
}
