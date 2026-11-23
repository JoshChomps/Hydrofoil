"""Regression tests for the 3-tier codebase audit and 3D lateral-directional split-flap turning."""

import numpy as np
import pytest

from hydrofoil.control.lqr import LQRController, allocate_split_flaps
from hydrofoil.design import apply_design, mass_properties
from hydrofoil.dynamics.rigid_body import RigidBody, strip_added_mass_sway_yaw
from hydrofoil.estimation.kalman import KalmanEstimator
from hydrofoil.forces.hull import HullHydrostatics
from hydrofoil.linearize import trim
from hydrofoil.monitor import FlightMonitor
from hydrofoil.params import DEFAULT_PARAMS, MANNED_PARAMS
from hydrofoil.sensors.models import SensorSuite
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import AiryWave, CalmWater


# --- Tier 1 unit & physics fixes --------------------------------------------

def test_airy_wave_clamps_above_surface_and_validates_inputs():
    w = AiryWave(amplitude=0.25, wavelength=5.0)
    v_surf = w.orbital_velocity(0.0, 0.0, 0.0, 0.0)
    v_air = w.orbital_velocity(0.0, 0.0, +2.0, 0.0)
    assert np.allclose(v_air, v_surf)
    with pytest.raises(ValueError):
        AiryWave(wavelength=0.0)
    with pytest.raises(ValueError):
        AiryWave(wavelength=5.0, period=-1.0)


def test_emerged_foil_has_zero_phantom_drag_and_pitching_moment():
    vp = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", sensors="ideal"))
    nu = np.array([8.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    tau_air = vp.left_foil.generalized_force(nu, delta=12.0, submergence_over_chord=-0.3, ventilation=1.0)
    assert np.allclose(tau_air, 0.0)
    assert vp.left_foil.last_CL == 0.0
    # Smoothly ramps up just below surface
    tau_shallow = vp.left_foil.generalized_force(nu, delta=12.0, submergence_over_chord=0.10)
    tau_deep = vp.left_foil.generalized_force(nu, delta=12.0, submergence_over_chord=3.0)
    assert 0.0 < abs(tau_shallow[0]) < abs(tau_deep[0])


def test_rigid_body_sway_yaw_added_mass_passivity_and_yaw_munk_moment():
    ms, iy = strip_added_mass_sway_yaw([(0.16, 0.90, 0.0), (0.12, 0.80, -2.0)])
    assert 20.0 < ms < 35.0 and 30.0 < iy < 45.0
    rb = RigidBody(275.0, np.diag([80.0, 260.0, 280.0]),
                   added_mass_heave=150.0, added_inertia_pitch=46.0, added_inertia_roll=64.0,
                   added_mass_sway=ms, added_inertia_yaw=iy)
    nu = np.array([8.0, 0.4, 0.2, 0.15, -0.1, 0.2])
    assert abs(float(nu @ rb.C(nu) @ nu)) < 1e-11
    nu_sway = np.array([8.0, 0.5, 0.0, 0.0, 0.0, 0.0])
    # Destabilizing yaw Munk moment -(C_A nu)_r = -Y_vdot * u * v
    assert -(rb.C(nu_sway) @ nu_sway)[5] == pytest.approx(-ms * 8.0 * 0.5, rel=1e-9)


def test_hull_displacement_sway_and_yaw_damping_vanish_when_foilborne():
    hull = HullHydrostatics(3.6, 1.2, 0.0, -0.35, 0.3)
    nu = np.array([2.0, 1.5, 0.0, 0.1, 0.1, 0.4])
    tau_wet = hull.drag(nu, 0.15, 120.0, 6000.0, hull_length=3.6, hump_amp=0.8)
    assert tau_wet[1] < -500.0 and tau_wet[5] < -500.0
    tau_dry = hull.drag(nu, 0.0, 120.0, 6000.0, hull_length=3.6, hump_amp=0.8)
    assert np.allclose(tau_dry, 0.0)


def test_split_front_flaps_produce_roll_and_adverse_yaw_while_back_wing_is_pure_pitch():
    vp = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", sensors="ideal"))
    x0, u0 = trim(vp, 8.0)
    nu = x0[6:]
    # Right roll initiation: delta_roll = -5 deg -> left > right -> +K (roll right) and -N (adverse yaw left)
    tau_l = vp.left_foil.generalized_force(nu, delta=u0[0] + 5.0, submergence_over_chord=3.2)
    tau_r = vp.right_foil.generalized_force(nu, delta=u0[0] - 5.0, submergence_over_chord=3.2)
    net = tau_l + tau_r
    assert net[3] > 500.0          # positive starboard roll torque
    assert net[5] < -20.0          # negative portward adverse yaw torque from higher left induced drag
    # Single-piece rear elevator creates pure pitch with zero roll or yaw in symmetric flow
    tau_e = vp.elev_foil.generalized_force(nu, delta=5.0, submergence_over_chord=3.0)
    assert abs(tau_e[3]) < 1e-9 and abs(tau_e[5]) < 1e-9 and abs(tau_e[4]) > 100.0


def test_wingtip_submergence_and_span_averaged_lift_loss_in_steep_bank():
    vp = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", sensors="ideal"))
    eta_lvl = np.array([0.0, 0.0, -0.85, 0.0, 0.0, 0.0])
    # Level: tip and effective submergence match center identically
    hc_c = vp._submergence_over_chord(vp.left_foil, eta_lvl, 0.0)
    assert vp._foil_tip_submergence_over_chord(vp.left_foil, eta_lvl, 0.0) == pytest.approx(hc_c)
    assert vp._effective_submergence_over_chord(vp.left_foil, eta_lvl, 0.0) == pytest.approx(hc_c)
    # Steep right bank (36 deg): high-side (left) center is still submerged, but outboard tip breaches above water
    eta_bank = np.array([0.0, 0.0, -0.85, np.radians(36.0), 0.0, 0.0])
    assert vp._submergence_over_chord(vp.left_foil, eta_bank, 0.0) > 0.5
    assert vp._foil_tip_submergence_over_chord(vp.left_foil, eta_bank, 0.0) < 0.0
    assert vp._effective_submergence_over_chord(vp.left_foil, eta_bank, 0.0) < vp._submergence_over_chord(vp.left_foil, eta_bank, 0.0)


def test_strut_sideslip_ventilation_triggers_and_recovers_with_hysteresis():
    vp = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", sensors="ideal"))
    s, _ = trim(vp, 8.0)
    # Impose 18 deg CG sideslip so local strut beta at mid-wetted length stays > 8.5 deg even after roll compliance
    s_skid = s.copy()
    t = 0.0
    zero = {"delta_main": -1.74, "delta_elev": 0.0, "delta_roll": 0.0}
    for _ in range(30):
        s_skid[6], s_skid[7] = 8.0, 8.0 * np.tan(np.radians(18.0))
        _, _, _, _, _ = vp.step(s_skid, t, CalmWater(), cmds_override=zero)
        t += vp.params["dt"]
    assert vp._vent["main_strut"] > 0.5
    # Return to zero sideslip: ventilation recovers smoothly
    for _ in range(150):
        s_clean = s.copy()
        _, _, _, _, _ = vp.step(s_clean, t, CalmWater(), cmds_override=zero)
        t += vp.params["dt"]
    assert vp._vent["main_strut"] < 0.1


def test_single_speed_lqr_kalman_and_sensor_rate_no_drift():
    vp = VehiclePhysics(dict(DEFAULT_PARAMS, controller="pid", sensors="ideal"))
    lqr1 = LQRController(vp, 0.01, speeds=(4.0,))
    K1, T1 = lqr1._sched(4.0)
    assert np.all(np.isfinite(K1)) and np.all(np.isfinite(T1))
    kf1 = KalmanEstimator(vp, 0.01, speeds=(4.0,))
    assert np.all(np.isfinite(kf1._interp(kf1.Ad, 4.0)))
    ss = SensorSuite(seed=0)
    st = vp.rest_state()
    for i in range(500):
        ss.measure(st, i * 0.01, 0.01)
    assert ss.channels["gyro"]._next >= 4.99


def test_split_flap_allocator_prioritizes_roll_within_physical_limit():
    m, r, mc, rc = allocate_split_flaps(16.0, 10.0, 20.0)
    assert r == pytest.approx(10.0) and m == pytest.approx(10.0)
    assert abs(m - r) <= 20.0 + 1e-9 and abs(m + r) <= 20.0 + 1e-9
    assert mc and not rc
    # Inside diamond: identity
    m2, r2, mc2, rc2 = allocate_split_flaps(5.0, -4.0, 20.0)
    assert m2 == pytest.approx(5.0) and r2 == pytest.approx(-4.0) and not mc2 and not rc2


def test_design_guards_mass_below_default_payload():
    p = apply_design(dict(MANNED_PARAMS), {"mass": 50.0})
    assert p["ixx"] > 5.0 and p["iyy"] > 20.0 and p["izz"] > 20.0
    with pytest.raises(ValueError):
        mass_properties(dict(MANNED_PARAMS, mass=50.0), None)


# --- Tier 2 & 3 closed-loop 3D turning & supervisor verification ------------

def test_lqr_coordinated_turn_tracks_15deg_bank_cleanly():
    p = dict(MANNED_PARAMS, controller="lqr", sensors="ideal", supervisor=False, target_speed=8.0)
    vp = VehiclePhysics(p)
    s, _ = trim(vp, 8.0)
    t = 0.0
    for _ in range(150):
        s, _, _, _, _ = vp.step(s, t, CalmWater()); t += p["dt"]
    p["phi_ref"] = 15.0
    z_ref = p["hull_offset"] - p["flying_height"]
    max_dz = 0.0
    for _ in range(800):
        s, c, _, dr, _ = vp.step(s, t, CalmWater()); t += p["dt"]
        assert dr <= 1e-6
        max_dz = max(max_dz, abs(s[2] - z_ref))
    assert abs(np.degrees(s[3]) - 15.0) < 0.1
    assert max_dz < 0.025                     # coordinated turn feedforward keeps height drop < 25 mm
    assert abs(vp.lqr.integ[1]) < 0.003       # roll integrator barely loaded (well below 0.03 clamp)
    assert np.degrees(s[5]) > 35.0            # sustained starboard heading turn
    # Steady-state split front flaps hold the low starboard wing UP (right > left) against spiral overbanking
    assert c["delta_roll"] > 1.5


def test_supervisor_safe_bank_limit_narrows_with_waves_and_ramps_smoothly():
    p = dict(MANNED_PARAMS, controller="lqr", sensors="noisy", supervisor=True)
    vp = VehiclePhysics(p)
    sup = vp.supervisor
    s, _ = trim(vp, 8.0)
    sup.pilot.update({"foil": True, "speed": 8.0, "height": 0.50, "bank": 0.0})
    sup.mode = "FOILING"
    sup.speed_ref = 8.0
    sup.height_ref = 0.50
    # Safe bank narrows cleanly with wave amplitude and low speed
    sup.wave_amp = 0.0
    b_calm8 = sup.safe_bank_limit(s)
    s_slow = s.copy(); s_slow[6] = 6.0
    b_calm6 = sup.safe_bank_limit(s_slow)
    sup.wave_amp = 0.25
    b_wave = sup.safe_bank_limit(s)
    assert b_calm8 == pytest.approx(15.0, abs=0.2)
    assert 11.5 <= b_calm6 <= 12.5
    assert b_wave < b_calm8 - 3.0

    # S-turn reversal (+12 -> -12 deg) stays smooth without sideslip slam or faults
    sup.wave_amp = 0.0
    t = 0.0
    for _ in range(150):
        s, _, _, _, _ = vp.step(s, t, CalmWater()); t += p["dt"]
    max_beta = 0.0
    for i in range(1400):
        if i == 0:
            sup.pilot["bank"] = 12.0
        elif i == 600:
            sup.pilot["bank"] = -12.0
        s, _, _, dr, _ = vp.step(s, t, CalmWater()); t += p["dt"]
        assert dr <= 1e-6
        max_beta = max(max_beta, abs(np.degrees(np.arctan2(s[7], max(s[6], 0.5)))))
    assert sup.mode == "FOILING" and not sup.faults
    assert max_beta < 4.5                     # ramped turn entry/reversal avoids >7 deg sideslip slams


def test_monitor_flags_wingtip_breach_and_high_sideslip_in_turns():
    p = dict(MANNED_PARAMS, controller="pid", sensors="ideal")
    vp = VehiclePhysics(p)
    mon = FlightMonitor(p)
    mon.established = True
    mon.air_since = 0.0
    s, _ = trim(vp, 8.0)
    cmds = {"delta_main": -1.7, "delta_elev": 0.0, "delta_roll": 0.0}
    # Banked to 28 deg (panel center > 1.4c, outer wingtip < 0.6c) + 7 deg sideslip
    s[3] = np.radians(28.0)
    s[7] = 8.0 * np.tan(np.radians(7.0))
    alerts = {a["id"]: a for a in mon.update(5.0, vp, s, cmds, CalmWater(), None, True)}
    assert "breach" in alerts and alerts["breach"]["level"] == "danger"
    assert "sideslip" in alerts and alerts["sideslip"]["level"] == "caution"
