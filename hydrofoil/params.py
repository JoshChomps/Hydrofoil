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
    # Vertical struts: give the side force that makes bank turn the boat instead of letting it
    # skid (without them a banked boat slides sideways and yaw has the wrong sign). Chord is a
    # PLACEHOLDER; the wetted length is computed from depth below the surface every step.
    "main_strut_chord": 0.12, "elev_strut_chord": 0.08,
    # Heave gains are tuned higher than sim.py's — here the vehicle starts at rest
    # and must generate real lift to take off, not just trim near an already-flying point.
    "kp_heave": 40.0, "ki_heave": 6.0, "kd_heave": 10.0,
    "kp_pitch": 1.0, "ki_pitch": 0.0, "kd_pitch": 0.1,
    # Ki must stay 0 here — any nonzero integral term reliably diverges under
    # wave disturbance (verified: Ki=0.05 blew up to 1000+ deg within 90s).
    "kp_roll":  2.0, "ki_roll":  0.0, "kd_roll":  0.2,
    # Target height of the HULL above the water once foilborne [m] (negative =
    # hull still below the surface). Internally converted to a CoG depth
    # reference via hull_offset — this is the number you actually care about.
    # Default = half the strut length (hull_offset + main_pos_z) / 2, so the
    # main strut's midpoint sits right at the waterline once flying — half
    # the shaft above water, half below.
    "flying_height": 0.55,
    "trim_ff_deg": 16.1, "trim_ff_speed": 4.0,   # speed-scheduled trim flap feed-forward (0 = off)
    "sensors": "ideal",                  # "ideal" (controllers read ground truth) or "noisy" (sensor models + Kalman estimator)
    "controller": "pid",                 # "pid" (three SISO loops) or "lqr" (scheduled MIMO, needs the servo actuator model)
    "phi_ref": 0.0, "theta_ref": 0.0,
    "waves_enabled": False,
    # Period is derived from wavelength by the deep-water dispersion relation. Direction is the
    # wave TRAVEL heading in degrees (0 = +x, the way the boat faces); oblique/beam seas (e.g. 90)
    # are what load the left and right foils differently and excite roll.
    "wave_amplitude": 0.05, "wave_wavelength": 5.0, "wave_direction": 0.0,
    "dt": 0.01, "t_end": 60.0,

    # Velocity (surge) controller: closed-loop speed control, thrust in Newtons.
    "target_speed": 4.0,
    "kp_speed": 30.0, "ki_speed": 5.0, "kd_speed": 0.0,
    "max_thrust": 50.0,   # matches the Unity TwinMotorController placeholder (50N)

    # Hull buoyancy / drag — a simplified wall-sided hull near the waterline.
    # hull_offset is how far above the CoG the hull rides (roughly the strut
    # length); waterplane_area/max_draft set how buoyant it is at rest and how
    # much draft it has before the linear approximation caps out.
    # Hull footprint (rectangle, PLACEHOLDER): waterplane area = length x beam = 0.6 m^2 as before. The footprint
    # (not just its area) sets roll/pitch hydrostatic stiffness through the waterplane inertia.
    "hull_length": 2.0, "hull_beam": 0.30, "hull_x": 0.0,    # m; footprint centre relative to the CoG (0 = floats level with the CoG over the CB)
    "max_draft": 0.3, "hull_offset": 0.6,
    "hull_drag_coeff": 20.0,
    # Roll/pitch damping of the floating hull [N*m*s/rad at full draft], scaled by wetted fraction. PLACEHOLDER
    # standing in for radiation damping (~0.2 of critical in pitch at rest draft).
    "hull_rot_damping": 400.0,

    # Control-surface drives (moteus r4.11 + BLDC + gearbox), see hydrofoil/actuation and docs/17.
    # Layout: "flap" = 40 % plain flap on each main-foil half (as drawn in the viewer);
    # "all_moving" = whole surface rotates about pivot_frac * chord from the LE.
    "main_actuation": "flap", "main_flap_frac": 0.4, "main_pivot_frac": 0.22,
    # The flap covers only the INBOARD half of each main-foil half-span (per the MAIN_FOIL_FLAP_EFFECTIVENESS
    # comment in forces/foil.py), so the hinge carries main_span * main_flap_span_frac of flap.
    "main_flap_span_frac": 0.5,
    "elev_actuation": "all_moving", "elev_pivot_frac": 0.22,   # PLACEHOLDER pivot just ahead of the AC (weathervane-stable)
    "main_surface_inertia": 4e-4, "elev_surface_inertia": 4e-4, # kg m^2 about the hinge, PLACEHOLDER (composite)
    "gear_ratio": 20.0,
    "v_bus": 40.0,                       # moteus supply [V] (10-44 V allowed); battery -> PDB -> motors is 40 V
    "supply_i_max": 25.0,                # PDB output limit [A] shared by everything on the rail
    # "servo": drives with dynamics, current/voltage limits and hinge moments in the loop (default);
    # "kinematic": legacy delay -> rate limit -> lag on the virtual channels (control/actuator.py).
    "actuator_model": "servo",
    "servo_params": None,                # overrides of actuation.servo.DEFAULT_SERVO_PARAMS (kp, kd, torque_cap, delay_s, ...)
    "flap_stop_deg": 30.0, "elev_stop_deg": 20.0,   # mechanical stops of the PHYSICAL surfaces, PLACEHOLDER
    "q_schedule_speed": 0.0,             # >0: scale flap commands by (v/u)^2 above this speed (gain scheduling on q); 0 = off
}
