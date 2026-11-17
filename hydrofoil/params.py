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
    # Fixed mounting incidence of the main foil [deg]. Carries most of the weight at cruise so the flap trims
    # near neutral and keeps authority both ways (at 0 the flap sat at 14 deg of a 20 deg limit at 4 m/s and
    # clipped in 10 cm waves). 2.5 deg puts the trim flap at about +12 deg at 3 m/s and -7 deg at 7 m/s.
    "main_incidence_deg": 2.5,
    # Centre-of-gravity shift from the nominal layout (dx forward, dy to starboard) [m], e.g. a rider's position.
    # VehiclePhysics.set_cg_offset() changes it mid-flight.
    "cg_offset": (0.0, 0.0),
    "elev_pos_x": -1.2, "elev_pos_z": 0.4,
    # Vertical struts: give the side force that makes bank turn the boat instead of letting it
    # skid (without them a banked boat slides sideways and yaw has the wrong sign). Chord is a
    # PLACEHOLDER; the wetted length is computed from depth below the surface every step.
    "main_strut_chord": 0.12, "elev_strut_chord": 0.08,
    # Heave gains are tuned higher than sim.py's — here the vehicle starts at rest
    # and must generate real lift to take off, not just trim near an already-flying point.
    "kp_heave": 40.0, "ki_heave": 6.0, "kd_heave": 10.0,
    "kp_pitch": 1.0, "ki_pitch": 1.0, "kd_pitch": 0.1,   # ki_pitch: holds theta = theta_ref (P-only sagged nose-down and stalled takeoff on short-tail / aft-foil plants)
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
    "trim_ff_deg": 1.63, "trim_ff_speed": 4.0,   # speed-scheduled trim flap feed-forward (0 = off); = linearize.trim(theta=0) main flap at trim_ff_speed (a test checks it)
    "sensors": "ideal",                  # "ideal" (controllers read ground truth) or "noisy" (sensor models + Kalman estimator)
    # "lqr" (default): scheduled MIMO LQR after takeoff (control/lqr.py, doc 19), needs the servo actuator model (falls
    # back to PID otherwise); "pid": the three SISO loops, which also fly takeoff. The PID gains below stay in use for takeoff.
    "controller": "lqr",
    "lqr_delay_s": None,                 # loop latency the LQR is designed for [s]; None = the drive model's delay_s
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


# Manned boat, 250-300 kg all-up (boat + rider), sized 2026-11-15 (docs/19, "Manned sizing"). Same layout as the
# model (T-foil, split flapped main foil at the CG, all-moving elevator aft), scaled: the main foil is 0.62 m^2
# (2 x 0.28 x 1.1 m) at 3 deg incidence so the trim flap is near neutral at the 8 m/s cruise and takeoff is
# about 5 m/s. The CG sits in the hull, 0.35 m above the hull bottom (hull_offset < 0), the foil 0.9 m under
# the surface and the hull 0.5 m clear of it when flying (1.4 m struts). Inertias, hull, damping, thrust and
# drive figures are PLACEHOLDERS scaled from the model, not measured.
MANNED_PARAMS: dict[str, Any] = dict(DEFAULT_PARAMS)
MANNED_PARAMS.update({
    "mass": 275.0,
    "ixx": 80.0, "iyy": 260.0, "izz": 280.0,
    # what the 275 kg is made of, for the boat editor (hydrofoil/design.py): the empty boat plus these point masses,
    # positions relative to the nominal CG [m] (x forward, y starboard, z up). PLACEHOLDER rider position.
    "default_payloads": [{"name": "Rider", "mass": 75.0, "x": -0.43, "y": 0.0, "z": 0.33}],
    "heave_damping": 600.0,
    "main_chord": 0.28, "main_span": 1.1, "main_lateral_offset": 0.60,
    "main_incidence_deg": 3.0,
    # Balanced flap (docs/20, adopted 2026-11-16): 25 % chord over the FULL half-span with the hinge set back 15 % of
    # the flap chord (thin theory overbalances near 33 %; 0.15 keeps more than 2x margin). Against the old 40 % chord,
    # half-span plain flap: about 4x less hinge moment and 3x less drive current in waves, deflections inside +-13 deg.
    "main_flap_frac": 0.25, "main_flap_span_frac": 1.0, "main_flap_setback": 0.15,
    # Drive train compliance and flap twist (docs/20 section 7). PLACEHOLDERS until the gearbox and flap are chosen:
    # flap GJ of a 70 x 13 mm carbon closed section (1.5 mm wall), elevator GJ of a 170 x 15 mm section (2 mm wall).
    "main_flap_GJ": 250.0, "elev_GJ": 1000.0,
    # Takeoff steps the flap to its 20 deg limit; unshaped that is 550-610 deg/s and saturates the drives' torque
    # (48 A, 17-21 A rail). Cruise in waves needs at most about 40 deg/s.
    "cmd_slew_deg_s": 150.0,
    "main_pos_x": 0.0, "main_pos_z": 1.75,
    "elev_chord": 0.17, "elev_span": 0.5, "elev_pos_x": -2.0, "elev_pos_z": 1.65,
    "main_strut_chord": 0.16, "elev_strut_chord": 0.12,
    "hull_offset": -0.35, "flying_height": 0.5,
    "hull_length": 3.6, "hull_beam": 1.2, "hull_x": 0.0, "max_draft": 0.3,
    "hull_drag_coeff": 120.0, "hull_rot_damping": 6000.0,
    "target_speed": 8.0, "max_thrust": 1000.0,
    "kp_speed": 360.0, "ki_speed": 60.0,
    # flap structural inertia about its hinge scales as span x c_f^3; the 9e-3 was for the 40 % half-span flap. Balanced
    # flap: x 0.488 for the geometry, x 0.617 for the axis 15 % into the flap -> 2.7e-3 (PLACEHOLDER until CAD)
    "main_surface_inertia": 2.7e-3, "elev_surface_inertia": 6e-3,
    # hinge moments reach 12-13 N m (takeoff, waves): the 10 N m firmware cap left the flap short and the boat
    # rolled over with a 10 cm lateral CG; 28 N m is just under the mj5208 x 20:1 peak (1.7 x 20 x 0.85)
    # Continuous drive current used by the sizing checks. The moteus runs above its 12 A no-heat-sink figure (20-32 A
    # quoted with heat sinking; the drives sit in water). PLACEHOLDER: the firmware co-dependency limit (coupled
    # current / torque / rail / temperature limits across the drives) will set the real value.
    "drive_i_cont": 20.0,
    # backlash / gear stiffness PLACEHOLDERS (economy 20:1 planetary: 10-20 arcmin, a few N m/arcmin). "dual": the
    # moteus takes position from an absolute encoder at the surface root (the free play is inside the loop) and its
    # damping from the motor's own velocity. Output-only feedback rang at takeoff with 72 N m through the gear (docs/20).
    "servo_params": {"torque_cap": 28.0, "backlash_deg": 0.25, "gear_stiffness": 4000.0, "encoder": "dual"},
    # heave filter for 8 m/s: encounter frequencies are lower (following / quartering 15 m waves about 1-1.5 rad/s);
    # a slower, larger wave band and a slower bias trade 13 mm calm wander (6 mm at the defaults) for 67-117 mm
    # instead of 90-260 mm rms in following seas (docs/19, "Manned sizing")
    "kf_wave": {"w0": 2.0, "zeta": 1.0, "sigma": 0.15}, "kf_heave": {"bias_walk": 0.005},
    "trim_ff_deg": 4.11, "trim_ff_speed": 6.0,   # = trim flap at 6 m/s for the balanced flap (was 6.57 on the 40 % flap)
    # The geometry the PID gains and LQR weights below were autotuned on; hydrofoil/consistency.py warns when the
    # boat no longer matches (re-run hydrofoil.tuning.autotune and update this).
    # 2026-11-16, balanced flap + compliant drives + slew limit: both sets RE-CHECKED on this geometry and kept. The
    # LQR search (9.55 -> 9.16) only won on ideal sensors and was WORSE with noisy ones (25 -> 28 mm, flap rate 31 ->
    # 47 deg/s); the PID search broke the supervised takeoff (see the PID gains). docs/20 section 8.
    "tuned_for": {"main_flap_frac": 0.25, "main_flap_span_frac": 1.0, "main_flap_setback": 0.15, "gear_ratio": 20.0},
    # PID gains from the autotuner (2026-11-15): these fly takeoff (and everything in PID mode). The relay-test rule
    # sets (Ziegler-Nichols etc.) scored best with perfect sensors but held the drives at their torque limit on noisy
    # ones; with drive effort charged properly the search kept gains near the model's. Following sea 95 -> 64 mm rms,
    # rider shift peak 183 -> 113 mm, takeoff settle 15.1 -> 13.8 s.
    # 2026-11-16, balanced flap: re-checked, KEPT. The autotuner's set (kp/ki/kd_heave 340 / 264 / 19) scored far
    # better on its five scenarios but stalls the SUPERVISED takeoff: below liftoff speed the supervisor holds the
    # height at the hull, the stiff heave loop drives the flap to -13.5 deg to do it, the hull digs in and the boat
    # stays on the drag hump at 5.3 m/s (tests/test_supervisor.py). The tuner's takeoff runs without the supervisor
    # (docs/20 section 8). Scaling these gains by the effectiveness ratio (0.21/0.342) scored worse (25.7 vs 22.1).
    "kp_heave": 44.94, "ki_heave": 6.488, "kd_heave": 7.906,
    "kp_pitch": 1.818, "ki_pitch": 1.038, "kd_pitch": 0.0987,
    "kp_roll": 2.0, "kd_roll": 0.191,
    # LQR weights from the autotuner (hydrofoil/tuning, docs/19 "Autotuner"), 2026-11-15: Bryson's rule, 4 rounds of
    # re-weighting, then 6 rounds of search over 5 scenarios incl. noisy sensors. Acceptable excursions, overriding
    # control.lqr.DEFAULT_WEIGHTS: tighter on height, heave rate and the height integral, looser on roll and pitch
    # rate, and a 2.3x larger command-rate allowance. Head / following sea height rms 27 / 36 -> 18 / 18 mm.
    "lqr_weights": {"z": 0.01084, "phi": 0.07882, "theta": 0.0348, "w": 0.08196, "q": 0.2391, "iz": 0.01292,
                    "du_main": 566.2, "du_elev": 566.2, "du_roll": 566.2},
    "lqr_speeds": (4.5, 5.5, 6.5, 8.0, 9.5), "lqr_engage_speed": 4.8, "est_engage_speed": 4.5,
})
