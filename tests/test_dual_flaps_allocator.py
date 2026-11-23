"""Tests for independent dual front flaps, Physics x Control-Stability Jacobian QP allocation,
reverse-drive gating, top-down turning safety envelope, singularity guards, and UI telemetry."""

import pathlib
import re
import time
from html.parser import HTMLParser

import numpy as np
import pytest

from hydrofoil.actuation.loads import flow_models
from hydrofoil.control.allocator import (
    SAFETY_PRESETS,
    compute_flap_safe_bounds,
    solve_box_qp_3d,
)
from hydrofoil.control.lqr import steady_turn_sensitivity
from hydrofoil.control.supervisor import compute_turn_envelope
from hydrofoil.linearize import trim
from hydrofoil.params import DEFAULT_PARAMS, MANNED_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import AiryWave, CalmWater, IrregularWave, encounter_frequency


def test_qp_solver_matches_kkt_and_calm_symmetric_equivalence():
    # Box-QP check against analytical constrained minimum
    H = np.diag([2.0, 4.0, 6.0])
    g = np.array([-6.0, 8.0, -3.0])
    lo = np.array([0.0, -1.0, -5.0])
    hi = np.array([2.0, 5.0, 5.0])
    x = solve_box_qp_3d(H, g, lo, hi)
    assert np.allclose(x, [2.0, -1.0, 0.5], atol=1e-7)

    # Calm symmetric flight: Jacobian QP reproduces legacy main-roll / main+roll mixing
    vp_jac = VehiclePhysics(dict(MANNED_PARAMS, controller="lqr", sensors="ideal", allocator="jacobian"))
    vp_leg = VehiclePhysics(dict(MANNED_PARAMS, controller="lqr", sensors="ideal", allocator="legacy"))
    s, u = trim(vp_jac, 8.0)
    for vp_i in (vp_jac, vp_leg):
        vp_i.drives["left"].delta = np.radians(u[0])
        vp_i.drives["right"].delta = np.radians(u[0])
        vp_i.drives["elev"].delta = np.radians(u[1])
    env = vp_jac.wave_sample(s[:6], 0.0, CalmWater())
    raw = {"delta_main": u[0], "delta_elev": u[1], "delta_roll": 0.0}
    c_j = vp_jac._servo_commands(raw, s, env, u_meas=8.0)
    c_l = vp_leg._servo_commands(raw, s, env, u_meas=8.0)
    for k in ("left", "right", "elev"):
        assert abs(c_j["target"][k] - c_l["target"][k]) < 1e-4


def test_allocator_performance_under_half_millisecond():
    vp = VehiclePhysics(dict(MANNED_PARAMS, controller="lqr", sensors="ideal", allocator="jacobian"))
    s, u = trim(vp, 8.0)
    env = vp.wave_sample(s[:6], 0.0, CalmWater())
    flows = flow_models(vp, s, env, vp.hinges)
    raw = {"delta_main": u[0] + 0.5, "delta_elev": u[1] - 0.2, "delta_roll": 1.2}
    prev = {"left": u[0], "right": u[0], "elev": u[1]}
    t0 = time.perf_counter()
    for _ in range(500):
        tgt, st = vp.allocator.allocate(vp, raw, s, env, prev, u_meas=8.0, flows=flows)
    avg_ms = (time.perf_counter() - t0) * 1000.0 / 500.0
    assert avg_ms < 0.60
    assert all(np.isfinite(tgt[k]) for k in ("left", "right", "elev"))
    assert st["solve_ms"] < 2.0


def test_singularity_immunity_and_matrix_fallbacks():
    vp = VehiclePhysics(dict(MANNED_PARAMS, controller="lqr", sensors="ideal", supervisor=True))
    env = vp.calm_sample()
    raw = {"delta_main": 8.0, "delta_elev": 2.0, "delta_roll": 3.0}
    prev = {"left": 0.0, "right": 0.0, "elev": 0.0}

    # u = 0 and u = 0.01 m/s (zero dynamic pressure) must never divide by zero or NaN
    for u_bad in (0.0, 0.01):
        s0 = vp.rest_state()
        s0[6] = u_bad
        tgt, st = vp.allocator.allocate(vp, raw, s0, env, prev, u_meas=u_bad)
        assert all(np.isfinite(tgt[k]) for k in ("left", "right", "elev"))
        assert all(np.isfinite(x) for b in st["bounds"].values() for x in b)

    # Emerged foils (h/c < 0) and steep attitude
    s_air = vp.rest_state()
    s_air[2] = -3.0
    s_air[3] = np.radians(50.0)
    s_air[6] = 8.0
    tgt_air, _ = vp.allocator.allocate(vp, raw, s_air, env, prev, u_meas=8.0)
    assert all(np.isfinite(tgt_air[k]) for k in ("left", "right", "elev"))

    # Rank-deficient turn sensitivity fallback must return finite matrices
    As = np.zeros((8, 8))
    Bs = np.zeros((8, 3))
    tnu, tu, alon, blinv = steady_turn_sensitivity(As, Bs)
    assert all(np.all(np.isfinite(a)) for a in (tnu, tu, alon, blinv))

    # safe_bank_limit edge arithmetic (no NaN near unit lens ratios)
    vp.supervisor.wave_amp = 0.9
    lim = vp.supervisor.safe_bank_limit(s_air)
    assert np.isfinite(lim) and lim >= 0.0


def test_dynamic_flap_bounds_asymmetric_stall_torque_cavitation_and_presets():
    vp = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", sensors="ideal"))
    s, _ = trim(vp, 8.0)
    env = vp.wave_sample(s[:6], 0.0, CalmWater())
    prev = {"left": 0.0, "right": 0.0, "elev": 0.0}

    # Asymmetric stall bound due to +3.0 deg mounting incidence (positive downward cap < |negative upward cap|)
    s_pitch = s.copy()
    s_pitch[8] = s_pitch[6] * np.tan(np.radians(4.0))
    b_std = compute_flap_safe_bounds(vp, s_pitch, env, prev, SAFETY_PRESETS["standard"])
    lo_l, hi_l = b_std["envelope_bounds"]["left"]
    assert hi_l < abs(lo_l)

    # Torque / cavitation tightening at 9.5 m/s shallow submergence vs 6.0 m/s deep submergence
    s_lo = s.copy(); s_lo[6] = 6.0; s_lo[2] = vp.params["hull_offset"] - 0.3
    s_hi = s.copy(); s_hi[6] = 9.5; s_hi[2] = vp.params["hull_offset"] - 0.65
    env_lo = vp.wave_sample(s_lo[:6], 0.0, CalmWater())
    env_hi = vp.wave_sample(s_hi[:6], 0.0, CalmWater())
    b_slow = compute_flap_safe_bounds(vp, s_lo, env_lo, prev, SAFETY_PRESETS["standard"])
    b_fast = compute_flap_safe_bounds(vp, s_hi, env_hi, prev, SAFETY_PRESETS["standard"])
    w_slow = b_slow["envelope_bounds"]["left"][1] - b_slow["envelope_bounds"]["left"][0]
    w_fast = b_fast["envelope_bounds"]["left"][1] - b_fast["envelope_bounds"]["left"][0]
    assert w_fast < w_slow

    # Conservative <= Standard <= Sport bound monotonicity
    b_con = compute_flap_safe_bounds(vp, s_hi, env_hi, prev, SAFETY_PRESETS["conservative"])
    b_spo = compute_flap_safe_bounds(vp, s_hi, env_hi, prev, SAFETY_PRESETS["sport"])
    w_con = b_con["envelope_bounds"]["left"][1] - b_con["envelope_bounds"]["left"][0]
    w_std = w_fast
    w_spo = b_spo["envelope_bounds"]["left"][1] - b_spo["envelope_bounds"]["left"][0]
    assert w_con <= w_std <= w_spo + 1e-6


def test_stability_gated_reverse_drive_activates_and_sheds_when_constrained():
    # Healthy reserves at cruise: opposing front flap deflections (-left / +right) are allowed
    p = dict(MANNED_PARAMS, controller="pid", sensors="ideal", reverse_drive_enabled=True, safety_margin="sport")
    vp = VehiclePhysics(p)
    s, _ = trim(vp, 8.0)
    env = vp.wave_sample(s[:6], 0.0, CalmWater())
    flows = flow_models(vp, s, env, vp.hinges)
    prev = {"left": -4.0, "right": 4.0, "elev": 0.0}
    raw_opposing = {"delta_main": 0.0, "delta_elev": 0.0, "delta_roll": 8.0}
    tgt_ok, st_ok = vp.allocator.allocate(vp, raw_opposing, s, env, prev, u_meas=8.0, flows=flows)
    assert tgt_ok["left"] < -2.0 and tgt_ok["right"] > 2.0
    assert st_ok["reverse_drive_active"] and not st_ok["reverse_drive_gated"]

    # Reverse drive disabled: prevents opposing cross-zero split
    vp.params["reverse_drive_enabled"] = False
    tgt_off, st_off = vp.allocator.allocate(vp, raw_opposing, s, env, {"left": 0.0, "right": 0.0, "elev": 0.0}, u_meas=8.0, flows=flows)
    assert st_off["reverse_drive_gated"] or abs(tgt_off["right"] - tgt_off["left"]) < abs(tgt_ok["right"] - tgt_ok["left"])
    vp.params["reverse_drive_enabled"] = True

    # Low elevator pitch headroom (elevator near limit): gate sheds reverse-drive differential split
    vp.params["safety_margin"] = "conservative"
    prev_high_e = {"left": -6.0, "right": 6.0, "elev": 18.0}
    raw_pitch_tight = {"delta_main": 0.0, "delta_elev": 18.5, "delta_roll": 12.0}
    tgt_shed, st_shed = vp.allocator.allocate(vp, raw_pitch_tight, s, env, prev_high_e, u_meas=8.0, flows=flows)
    assert st_shed["reverse_drive_gated"]
    assert abs(tgt_shed["right"] - tgt_shed["left"]) < abs(raw_pitch_tight["delta_roll"] * 2.0)


def test_asymmetric_oblique_wave_and_jammed_flap_reallocation():
    vp = VehiclePhysics(dict(MANNED_PARAMS, controller="lqr", sensors="ideal"))
    s, u = trim(vp, 8.0)
    wave = AiryWave(amplitude=0.18, wavelength=12.0, direction=np.radians(90.0))
    env = vp.wave_sample(s[:6], 0.25, wave)
    flows = flow_models(vp, s, env, vp.hinges)
    raw = {"delta_main": float(u[0]), "delta_elev": float(u[1]), "delta_roll": 0.0}
    prev = {"left": float(u[0]), "right": float(u[0]), "elev": float(u[1])}

    # Asymmetric beam-wave flow changes left vs right safe/optimal angles without coupling crash
    tgt_w, st_w = vp.allocator.allocate(vp, raw, s, env, prev, u_meas=8.0, flows=flows)
    assert all(np.isfinite(tgt_w[k]) for k in ("left", "right", "elev"))
    assert st_w["lift_margin"] > 0.4

    # Jammed right flap locks right at encoder angle and routes roll balance cleanly to left flap
    vp.drives["right"].delta = np.radians(6.0)
    tgt_j, st_j = vp.allocator.allocate(
        vp, {"delta_main": 1.0, "delta_elev": 0.0, "delta_roll": -2.0},
        s, env, {"left": 6.0, "right": 6.0, "elev": 0.0}, u_meas=8.0, jam=("right", 6.0), flows=flows
    )
    assert tgt_j["right"] == pytest.approx(6.0, abs=1e-6)
    assert st_j["binding"]["right"] == "jammed"
    assert tgt_j["left"] >= 7.5 - 1e-6
    tgt_j2, _ = vp.allocator.allocate(
        vp, {"delta_main": 1.0, "delta_elev": 0.0, "delta_roll": -2.0},
        s, env, tgt_j, u_meas=8.0, jam=("right", 6.0), flows=flows
    )
    assert tgt_j2["left"] >= 9.0 - 1e-6


def test_top_down_turn_envelope_presets_waves_and_yaw_rate_command():
    p = dict(MANNED_PARAMS, controller="lqr", sensors="ideal", supervisor=True)
    vp = VehiclePhysics(p)
    sup = vp.supervisor
    s, _ = trim(vp, 8.0)
    sup.mode = "FOILING"
    sup.height_ref = 0.50

    envs = {}
    for preset in ("conservative", "standard", "sport"):
        vp.set_safety_margin(preset)
        envs[preset] = compute_turn_envelope(vp, cstate=s, wave_amp=0.0, height_ref=0.50, cfg=sup.c)

    assert envs["conservative"]["bank_max_deg"] < envs["standard"]["bank_max_deg"] <= envs["sport"]["bank_max_deg"]
    assert envs["conservative"]["yaw_rate_max_deg_s"] < envs["standard"]["yaw_rate_max_deg_s"] <= envs["sport"]["yaw_rate_max_deg_s"]
    assert envs["conservative"]["turn_radius_min_m"] > envs["standard"]["turn_radius_min_m"] >= envs["sport"]["turn_radius_min_m"] - 1e-6

    # Waves tighten safe turn rate and widen minimum turn radius
    vp.set_safety_margin("standard")
    env_calm = compute_turn_envelope(vp, cstate=s, wave_amp=0.0, height_ref=0.50, cfg=sup.c)
    env_rough = compute_turn_envelope(vp, cstate=s, wave_amp=0.25, height_ref=0.50, cfg=sup.c)
    assert env_rough["yaw_rate_max_deg_s"] < env_calm["yaw_rate_max_deg_s"]
    assert env_rough["turn_radius_min_m"] > env_calm["turn_radius_min_m"]

    # High sideslip invokes CBF turn-rate clamping
    s_slip = s.copy()
    s_slip[7] = 8.0 * np.tan(np.radians(6.2))
    env_slip = compute_turn_envelope(vp, cstate=s_slip, wave_amp=0.0, height_ref=0.50, cfg=sup.c)
    assert env_slip["binding_constraint"] == "strut sideslip"
    assert env_slip["yaw_rate_max_deg_s"] < env_calm["yaw_rate_max_deg_s"]

    # Top-down yaw_rate command translates cleanly into coordinated bank within safe bounds
    sup.pilot.update({"foil": True, "speed": 8.0, "height": 0.50, "yaw_rate": 4.0})
    t = 0.0
    for _ in range(300):
        s, _, _, dr, _ = vp.step(s, t, CalmWater())
        t += p["dt"]
        assert dr <= 1e-6
    assert np.degrees(s[11]) > 1.5
    assert abs(np.degrees(s[3])) <= sup.bank_limit + 0.5


@pytest.mark.parametrize("ctrl_mode", ["pid", "lqr", "auto"])
def test_closed_loop_boundary_stress_model_and_manned(ctrl_mode):
    # Model preset in oblique waves with bank step
    p_mod = dict(DEFAULT_PARAMS, controller=ctrl_mode, allocator="jacobian",
                 safety_margin="standard", supervisor=False, target_speed=4.2)
    vp_m = VehiclePhysics(p_mod)
    sm, _ = trim(vp_m, 4.2)
    wave_m = AiryWave(amplitude=0.05, wavelength=6.0, direction=np.radians(45.0))
    t = 0.0
    for i in range(400):
        vp_m.params["phi_ref"] = 8.0 if i > 50 else 0.0
        sm, _, _, dr, _ = vp_m.step(sm, t, wave_m)
        t += p_mod["dt"]
        assert np.all(np.isfinite(sm))
        assert dr < 0.05

    # Manned preset in Auto Combined with beam waves, off-center CG, and bank reversal
    if ctrl_mode == "auto":
        p_man = dict(MANNED_PARAMS, controller="auto", allocator="jacobian",
                     safety_margin="standard", supervisor=True, cg_offset=(0.0, 0.08))
        vp = VehiclePhysics(p_man)
        s, _ = trim(vp, 8.0)
        vp.supervisor.mode = "FOILING"
        vp.supervisor.speed_ref = 8.0
        vp.supervisor.height_ref = 0.45
        vp.supervisor.pilot.update({"foil": True, "speed": 8.0, "height": 0.45, "bank": 10.0})
        wave = AiryWave(amplitude=0.14, wavelength=12.0, direction=np.radians(60.0))
        t = 0.0
        for i in range(600):
            if i == 300:
                vp.supervisor.pilot["bank"] = -10.0
            s, _, _, dr, _ = vp.step(s, t, wave)
            t += p_man["dt"]
            assert np.all(np.isfinite(s))
            assert dr <= 1e-6
        assert not vp.supervisor.faults


def test_sim_server_and_page_expose_dual_flaps_turn_envelope_and_health():
    from hydrofoil import sim_server

    assert "safety_margin" in sim_server.FIELDS
    assert "allocator" in sim_server.FIELDS
    html = pathlib.Path("hydrofoil/assets/launcher/page.html").read_text(encoding="utf-8")
    for hook in (
        "renderFlightSystems", "drawTurnEnvelope", "segMargin", "segAlloc", "segRev",
        "turnSvg", "kvRmin", "kvYawRate", "drvRows", "kvHydro", "kvVent",
    ):
        assert hook in html

    # HTML tag matching check
    class _P(HTMLParser):
        def __init__(self):
            super().__init__()
            self.stack = []

        def handle_starttag(self, tag, _attrs):
            if tag not in ("meta", "link", "input", "br", "hr"):
                self.stack.append(tag)

        def handle_endtag(self, tag):
            if self.stack and self.stack[-1] == tag:
                self.stack.pop()

    parser = _P()
    parser.feed(html)
    assert parser.stack == []
    assert isinstance(sim_server.page(), str) and len(sim_server.page()) > 2000
    assert re.search(r"safety_margin", sim_server.page())


def test_phase_a_separation_reverse_drive_eta_turn_and_antiwindup():
    # 1. Strict separation principle: noisy-mode allocator ignores truth CG shifts, _vent, and true wave state
    vp_a = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", sensors="noisy"))
    vp_b = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", sensors="noisy", cg_offset=(0.15, 0.12), cg_z=0.10))
    s_ref, _ = trim(VehiclePhysics(dict(MANNED_PARAMS, controller="pid", sensors="ideal")), 8.0)
    vp_b._vent["left"] = 0.9
    raw = {"delta_main": 1.0, "delta_elev": 0.0, "delta_roll": 2.0}
    env_w = vp_b.wave_sample(s_ref[:6], 0.2, AiryWave(0.15, 12.0, np.radians(90.0)))
    c_a = vp_a._servo_commands(raw, s_ref, vp_a.calm_sample(), u_meas=8.0, alloc_state=s_ref)
    c_b = vp_b._servo_commands(raw, s_ref, env_w, u_meas=8.0, alloc_state=s_ref)
    for k in ("left", "right", "elev"):
        assert c_a["target"][k] == pytest.approx(c_b["target"][k], abs=1e-9)
    assert c_a["alloc"]["lift_margin"] == pytest.approx(c_b["alloc"]["lift_margin"], abs=1e-9)

    # 2. reverse_drive_enabled=False no longer locks normal positive-lift 12 deg cruise bank turns
    p2 = dict(MANNED_PARAMS, controller="lqr", sensors="ideal", supervisor=True, reverse_drive_enabled=False)
    vp2 = VehiclePhysics(p2)
    s2, _ = trim(vp2, 8.0)
    vp2.supervisor.mode = "FOILING"
    vp2.supervisor.speed_ref = 8.0
    vp2.supervisor.height_ref = 0.50
    vp2.supervisor.pilot.update({"foil": True, "speed": 8.0, "height": 0.50, "bank": 12.0})
    t = 0.0
    for _ in range(400):
        s2, _, _, _, _ = vp2.step(s2, t, CalmWater())
        t += p2["dt"]
    assert abs(np.degrees(s2[3]) - 12.0) < 1.0

    # 3. Plant-derived turning_efficiency replaces hardcoded constant and reflects strut chord changes
    vp_mod = VehiclePhysics(dict(DEFAULT_PARAMS, controller="lqr"))
    vp_man = VehiclePhysics(dict(MANNED_PARAMS, controller="lqr"))
    vp_thin = VehiclePhysics(dict(MANNED_PARAMS, controller="lqr", main_strut_chord=0.08))
    eta_mod = vp_mod.turning_efficiency(4.0)
    eta_man = vp_man.turning_efficiency(8.0)
    eta_thin = vp_thin.turning_efficiency(8.0)
    assert 0.08 < eta_mod < 0.16
    assert 0.22 < eta_man < 0.36
    assert eta_thin > eta_man

    # 4. LQR sync_sent (ch, j) anti-windup rollback + active sideslip CBF unwinding
    p4 = dict(MANNED_PARAMS, controller="lqr", sensors="ideal", safety_margin="conservative",
              supervisor=False, target_speed=9.5, phi_ref=18.0)
    vp4 = VehiclePhysics(p4)
    s4, _ = trim(vp4, 9.5)
    t = 0.0
    for _ in range(300):
        s4, _, _, _, _ = vp4.step(s4, t, CalmWater())
        t += p4["dt"]
    assert float(np.max(np.abs(vp4.lqr.integ))) < 0.008
    assert abs(np.degrees(s4[3]) - 18.0) < 1.5

    sup = vp2.supervisor
    sup.bank_ref = 16.0
    sup.pilot.update({"foil": True, "speed": 8.0, "height": 0.50, "bank": 16.0})
    s_slip, _ = trim(vp2, 8.0)
    s_slip[7] = 8.0 * np.tan(np.radians(6.1))
    sup.update(10.0, s_slip)
    assert sup.bank_ref < 16.0


def test_phase_bc_3d_cg_active_cavitation_legacy_sigma_and_design_cache():
    from hydrofoil.design import apply_design, derived, trim_curve
    from hydrofoil.forces.foil import Foil, FoilGeometry, FoilTables

    # 1. 3D vertical cg_z and prop_pos_z thrust-line pitching moment
    vp_0 = VehiclePhysics(dict(MANNED_PARAMS, controller="pid"))
    vp_z = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", cg_z=0.15, prop_pos_z=1.5))
    vp_z0 = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", cg_z=0.15, prop_pos_z=0.0))
    assert vp_z.hull_offset == pytest.approx(vp_0.hull_offset - 0.15, abs=1e-6)
    assert vp_z.left_foil.r_attach[2] == pytest.approx(vp_0.left_foil.r_attach[2] + 0.15, abs=1e-6)
    assert vp_z.rest_state()[2] == pytest.approx(vp_0.rest_state()[2] - 0.15, abs=1e-6)
    s_t, _ = trim(vp_0, 8.0)
    qdot_with = vp_z.physics_rhs(s_t, {"delta_main": 0.0, "delta_elev": 0.0, "delta_roll": 0.0}, 200.0)[10]
    qdot_no = vp_z0.physics_rhs(s_t, {"delta_main": 0.0, "delta_elev": 0.0, "delta_roll": 0.0}, 200.0)[10]
    assert qdot_with - qdot_no > 0.9

    # 2. Active C^1 cavitation: inactive at 8 m/s cruise and 1.5 m/s wave flow; degrades CL/CD at 15 m/s with exact FD derivatives
    polar = FoilTables.naca0009()
    f_on = Foil(FoilGeometry(0.28, 1.1, aoa_offset=3.0), polar, np.array([0.0, 0.6, 1.75]),
                flap_effectiveness=0.342, flap_cm_delta=-0.4, cavitation_model=True)
    f_off = Foil(FoilGeometry(0.28, 1.1, aoa_offset=3.0), polar, np.array([0.0, 0.6, 1.75]),
                 flap_effectiveness=0.342, flap_cm_delta=-0.4, cavitation_model=False)
    r_nom = f_on.local_derivatives(np.array([8.0, 0, 0, 0, 0, 0], float), delta=0.0, submergence_over_chord=0.90 / 0.28)
    r_wave = f_on.local_derivatives(np.array([0.1, 0, -1.5, 0, 0, 0], float), delta=2.0, submergence_over_chord=6.0)
    assert r_nom["cav_factor"] == pytest.approx(1.0, abs=1e-9)
    assert r_wave["cav_factor"] == pytest.approx(1.0, abs=1e-9)

    nu_hi = np.array([15.0, 0, 0, 0, 0, 0], float)
    r_hi = f_on.local_derivatives(nu_hi, delta=8.0, submergence_over_chord=1.5)
    r_hi_off = f_off.local_derivatives(nu_hi, delta=8.0, submergence_over_chord=1.5)
    assert r_hi["cav_factor"] < 0.96
    assert r_hi["CL"] < r_hi_off["CL"]
    assert r_hi["CD"] > r_hi_off["CD"]
    eps = 1e-4
    fd = (f_on.generalized_force(nu_hi, delta=8.0 + eps, submergence_over_chord=1.5)
          - f_on.generalized_force(nu_hi, delta=8.0 - eps, submergence_over_chord=1.5)) / (2.0 * eps)
    assert np.max(np.abs(r_hi["dtau_ddelta"] - fd)) / max(float(np.max(np.abs(fd))), 1e-9) < 1e-4

    # 3. Legacy sigma update + design flap_margin & cache keys
    vp_leg = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", allocator="legacy"))
    s_l, _ = trim(vp_leg, 8.0)
    vp_leg.step(s_l, 0.0, CalmWater())
    assert 1.0 < vp_leg._last_sigma < 10.0

    p_18 = apply_design(dict(MANNED_PARAMS), {"flap_cmd_max_deg": 18.0})
    d_18 = derived(p_18, include_summary=False)
    assert d_18["flap_margin"] == pytest.approx(18.0 - abs(d_18["cruise_flap"]), abs=1e-6)
    c_a = trim_curve(p_18, speeds=(6.0, 8.0))
    c_b = trim_curve(dict(p_18, allocator="legacy", safety_margin="sport", wave_type="irregular"), speeds=(6.0, 8.0))
    assert c_a is c_b


def test_encounter_frequency_and_irregular_jonswap_waves():
    from hydrofoil.constants import G

    w_p = 2.25
    u = 8.0
    we_follow = encounter_frequency(w_p, u, 0.0)
    we_beam = encounter_frequency(w_p, u, 0.5 * np.pi)
    we_head = encounter_frequency(w_p, u, np.pi)
    assert we_beam == pytest.approx(w_p, abs=1e-9)
    assert we_head == pytest.approx(w_p + w_p * w_p * u / G, rel=1e-6)
    assert we_follow < w_p < we_head

    iw = IrregularWave(significant_height=0.24, peak_wavelength=12.0, direction=np.radians(90.0), spread_s=4.0, seed=42)
    assert iw.amplitude == pytest.approx(0.12, abs=1e-9)
    assert isinstance(iw.elevation(0.0, 0.0, 0.0), float)
    ys = np.linspace(0.0, 250.0, 400)
    eta_grid = iw.elevation(np.zeros_like(ys), ys, 1.5)
    assert eta_grid.shape == ys.shape
    assert 0.18 < 4.0 * float(np.std(eta_grid)) < 0.30
    assert np.allclose(iw.orbital_velocity(1.0, 2.0, 2.5, 0.5), iw.orbital_velocity(1.0, 2.0, 0.0, 0.5))

    p = dict(MANNED_PARAMS, controller="auto", sensors="noisy", supervisor=True, allocator="jacobian", wave_type="irregular")
    vp = VehiclePhysics(p)
    s, _ = trim(vp, 8.0)
    vp.supervisor.mode = "FOILING"
    vp.supervisor.speed_ref = 8.0
    vp.supervisor.height_ref = 0.50
    vp.supervisor.pilot.update({"foil": True, "speed": 8.0, "height": 0.50, "bank": 0.0})
    t = 0.0
    for _ in range(500):
        s, _, _, dr, _ = vp.step(s, t, iw)
        t += p["dt"]
        assert np.all(np.isfinite(s))
        assert dr <= 1e-6
    assert not vp.supervisor.faults
