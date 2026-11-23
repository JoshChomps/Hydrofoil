"""Tests for launcher/UI ergonomics, 3-way speed units, auto combined controller, 3D CG design calc, and autotuner persistence."""

import json
import numpy as np
import pytest

from hydrofoil.design import apply_design, derived, design_summary, mass_properties
from hydrofoil.params import DEFAULT_PARAMS, MANNED_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import CalmWater


def test_auto_combined_controller_rises_to_lqr_and_falls_back_to_pid():
    # Without supervisor: PID below takeoff speed / hull-wetted -> LQR when foilborne -> PID when landing/slowing
    p = dict(MANNED_PARAMS, controller="auto", sensors="ideal", supervisor=False, target_speed=8.0)
    vp = VehiclePhysics(p)
    assert vp.controller_status(0.0)["mode"] == "auto"
    assert not vp.lqr.engaged
    s, t = vp.rest_state(), 0.0
    dt = p["dt"]
    engaged_at = None
    for _ in range(int(18.0 / dt)):
        s, _, _, dr, _ = vp.step(s, t, CalmWater()); t += dt
        if vp.lqr.engaged and engaged_at is None:
            engaged_at = (t, dr)
    assert engaged_at is not None and engaged_at[1] <= 1e-3
    assert vp.controller_status(s[6])["active"] == "LQR"

    # Now command landing / slow down -> LQR must hand back smoothly to PID as speed falls / hull meets water
    p["target_speed"] = 2.0
    p["flying_height"] = -0.10
    handed_back = False
    for _ in range(int(12.0 / dt)):
        s, _, _, _, _ = vp.step(s, t, CalmWater()); t += dt
        if not vp.lqr.engaged and s[6] < 5.0:
            handed_back = True
            break
    assert handed_back
    assert abs(np.degrees(s[3])) < 2.0
    assert vp.controller_status(s[6])["active"] in ("LQR → PID", "PID", "PID (arming LQR)")
    for _ in range(int(0.8 / dt)):
        s, _, _, _, _ = vp.step(s, t, CalmWater()); t += dt
    assert vp.controller_status(s[6])["active"].startswith("PID")

    # Supervised full cycle: DISPLACEMENT -> TAKEOFF -> FOILING -> LANDING -> DISPLACEMENT
    ps = dict(MANNED_PARAMS, controller="auto", supervisor=True, sensors="ideal")
    vps = VehiclePhysics(ps)
    sup = vps.supervisor
    s, t = vps.rest_state(), 0.0
    saw_lqr = False
    for i in range(int(42.0 / dt)):
        if i == 300:
            sup.pilot.update({"foil": True, "speed": 7.5})
        if i == 2500:
            sup.pilot.update({"foil": False, "speed": 2.5})
        s, *_ = vps.step(s, t, CalmWater()); t += dt
        if vps.lqr.engaged:
            saw_lqr = True
    assert saw_lqr and sup.mode == "DISPLACEMENT" and not vps.lqr.engaged


def test_runtime_controller_mode_switching_and_pilot_keybinds():
    import mujoco
    from hydrofoil.viewer import Pilot, SLIDERS, IDX, build_mjcf, _ctrl_label

    p = dict(MANNED_PARAMS, controller="auto")
    vp = VehiclePhysics(p)
    assert vp.set_controller_mode("lqr") == "lqr" and not vp.lqr.auto_handback
    assert vp.set_controller_mode("pid") == "pid" and vp.lqr.force is False
    assert _ctrl_label(vp, 0.0) == "PID"
    assert vp.set_controller_mode("auto") == "auto" and vp.lqr.auto_handback and vp.lqr.force is None

    ctrl = np.zeros(len(SLIDERS))
    pilot = Pilot(ctrl)
    assert pilot.unit_info()[0] == "m/s"
    pilot.key(ord("u"))
    assert pilot.unit_info()[0] == "km/h" and pilot.kmh
    pilot.key(ord("u"))
    assert pilot.unit_info()[0] == "mph"
    pilot.key(ord("u"))
    assert pilot.unit_info()[0] == "m/s"
    assert pilot.set_unit("mph") == "mph"

    # Space levels wings immediately; +/- and F/C/R/D keys set flags
    ctrl[IDX["bank_ref"]] = 10.0
    pilot.bank_spring = True
    pilot.key(32)
    assert ctrl[IDX["bank_ref"]] == 0.0 and not pilot.bank_spring
    pilot.key(ord("="))
    assert ctrl[IDX["speed_ref"]] > 0.0
    pilot.key(ord("f"))
    assert pilot.toggle_foil
    pilot.key(ord("c"))
    assert pilot.toggle_ctrl

    # Moved rider + custom payload compile cleanly into MuJoCo MJCF
    edited = apply_design(dict(MANNED_PARAMS), {
        "payloads": [
            {"name": "Rider", "mass": 80.0, "x": -0.25, "y": 0.12, "z": 0.35},
            {"name": "Battery", "mass": 25.0, "x": 0.40, "y": -0.10, "z": 0.10},
        ]
    })
    m = mujoco.MjModel.from_xml_string(build_mjcf(edited))
    assert m.nbody > 1


def test_design_idempotent_3d_cg_inertia_hull_shift_and_layman_summary():
    # Sequential empty_mass + placed 3D payloads stay idempotent and match exact parallel-axis math
    payloads = [
        {"name": "Rider", "mass": 80.0, "x": -0.30, "y": 0.15, "z": 0.40},
        {"name": "Battery", "mass": 20.0, "x": 0.50, "y": -0.15, "z": 0.10},
    ]
    p1 = apply_design(dict(MANNED_PARAMS), {"empty_mass": 190.0, "payloads": payloads})
    assert p1["mass_empty"] == pytest.approx(190.0)
    assert p1["mass"] == pytest.approx(290.0)
    assert p1["cg_offset"][1] != 0.0 and p1["cg_z"] != 0.0

    # Applying a hull edit after payload edits must not corrupt bare hull weight or re-subtract payloads
    p2 = apply_design(p1, {"hull_length": 4.0})
    assert p2["mass_empty"] == pytest.approx(190.0)
    assert p2["mass"] == pytest.approx(290.0)

    # Check VehiclePhysics also shifts HullHydrostatics cells on set_cg_offset
    vp = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", sensors="ideal"))
    tau_before = vp.hull.buoyancy(vp.rest_state(), np.zeros(vp.hull.n))
    vp.set_cg_offset((0.20, 0.10))
    tau_after = vp.hull.buoyancy(vp.rest_state(), np.zeros(vp.hull.n))
    assert abs(tau_after[4] - tau_before[4]) > 100.0
    assert abs(tau_after[3] - tau_before[3]) > 100.0

    # Qualitative summary reports good vs danger on impossible low-thrust boat
    d_ok = derived(p1)
    assert d_ok["summary"]["verdict"]["status"] in ("good", "caution")
    p_bad = apply_design(dict(MANNED_PARAMS), {"max_thrust": 50.0, "flying_height": 1.65})
    sm_bad = design_summary(derived(MANNED_PARAMS, include_summary=False), derived(p_bad, include_summary=False), p_bad)
    assert sm_bad["verdict"]["status"] == "danger"


def test_sim_server_autotune_persistence_and_ui_features(tmp_path, monkeypatch):
    from hydrofoil import sim_server

    state_dir = tmp_path / "local"
    monkeypatch.setattr(sim_server, "STATE_DIR", state_dir)
    monkeypatch.setattr(sim_server, "OVERRIDES", state_dir / "overrides.json")
    monkeypatch.setattr(sim_server, "TUNE_PROGRESS", state_dir / "tune_progress.json")
    monkeypatch.setattr(sim_server, "tune_proc", None)
    monkeypatch.setattr(sim_server, "_last_tune_data", {"status": "idle"})
    monkeypatch.setattr(sim_server, "_auto_saved_sig", None)

    state_dir.mkdir(parents=True)
    # Simulate an autotuner result completing in background -> tune_status must auto-persist to overrides.json
    prog = {
        "status": "done",
        "preset": "manned",
        "target": "pid",
        "elapsed_s": 12.3,
        "stages": [{"name": "current values", "cost": 22.0, "values": {}, "metrics": {}}],
        "best": {
            "cost": 16.5,
            "source": "search, round 2",
            "values": {"kp_heave": 71.25, "ki_heave": 7.1, "kd_heave": 8.0,
                       "kp_pitch": 3.2, "ki_pitch": 1.5, "kd_pitch": 0.1,
                       "kp_roll": 2.1, "kd_roll": 0.25},
            "metrics": {},
        },
    }
    (state_dir / "tune_progress.json").write_text(json.dumps(prog), encoding="utf-8")
    st = sim_server.tune_status("manned")
    assert st["auto_saved"] is True
    assert st["stored"]["has_tuned_pid"] is True
    assert st["stored"]["stored"]["pid"]["kp_heave"] == pytest.approx(71.25)
    saved = json.loads((state_dir / "overrides.json").read_text(encoding="utf-8"))
    assert saved["manned"]["kp_heave"] == pytest.approx(71.25)

    # Stored backend check when switching to model preset stays isolated to model
    st_mod = sim_server.tune_status("model")
    assert st_mod["stored"]["preset"] == "model" and not st_mod["stored"]["has_tuned_pid"]

    # Rendered HTML page contains auto mode, phase-aware tuning, 3 speed units, and drag mass instructions
    html = sim_server.page()
    for token in ("Auto Combined", "Phase-Aware Cruise", "mph", "km/h", "m/s", "Placed Weights", "Auto-Calculates 3D Center of Gravity"):
        assert token in html


def test_phase_aware_auto_tuning_tunes_rising_cruising_and_falling(tmp_path, monkeypatch):
    from hydrofoil import sim_server
    from hydrofoil.control.lqr import DEFAULT_WEIGHTS
    from hydrofoil.tuning.autotune import LQR_KEYS, PID_KEYS, candidate_params, tune
    from hydrofoil.tuning.scoring import phase_costs, phase_scenarios, run_scenario

    # 1. Phase-grouped scenarios include turn (cruising) and landing (falling)
    all_sc = phase_scenarios(MANNED_PARAMS)
    assert {"takeoff", "supervised", "head", "following", "shift", "noisy", "turn", "landing"} == set(all_sc)
    assert set(phase_scenarios(DEFAULT_PARAMS)) == {"takeoff", "head", "following", "shift", "noisy", "turn", "landing"}

    # 2. Both new scenarios execute cleanly under controller='auto'
    p_auto = dict(MANNED_PARAMS, controller="auto")
    m_turn = run_scenario(p_auto, "turn")
    assert not m_turn["failed"]
    assert m_turn["roll_rms_err"] < 2.0 and m_turn["sideslip_peak"] < 6.0 and m_turn["roll_overshoot"] < 5.0
    m_land = run_scenario(p_auto, "landing")
    assert not m_land["failed"] and m_land["handback_ok"] and m_land["vz_touch"] < 0.8

    # 3. candidate_params couples iphi/itheta to phi/theta multipliers relative to base/MANNED weights
    base_w = dict(DEFAULT_WEIGHTS, **(MANNED_PARAMS.get("lqr_weights") or {}))
    x = np.zeros(len(LQR_KEYS))
    x[LQR_KEYS.index("phi")] = np.log(0.5)
    x[LQR_KEYS.index("theta")] = np.log(2.0)
    cand = candidate_params(MANNED_PARAMS, "auto_lqr", x)
    assert cand["controller"] == "auto"
    assert cand["lqr_weights"]["phi"] == pytest.approx(0.5 * base_w["phi"])
    assert cand["lqr_weights"]["iphi"] == pytest.approx(0.5 * base_w["iphi"])
    assert cand["lqr_weights"]["theta"] == pytest.approx(2.0 * base_w["theta"])
    assert cand["lqr_weights"]["itheta"] == pytest.approx(2.0 * base_w["itheta"])

    # 4. End-to-end phase-aware autotuning produces combined PID + LQR values & per-phase score breakdown
    prog_path = tmp_path / "prog.json"
    res = tune("model", "auto", generations=1, pop=2, bryson_rounds=1, progress=prog_path)
    best = res["best"]
    assert best["cost"] <= res["baseline"]["cost"] + 1e-9
    assert set(best["phase_costs"]) == {"rising", "cruising", "falling", "total"}
    for k in PID_KEYS:
        assert k in best["values"]
    assert "lqr_weights" in best["values"] and "phi" in best["values"]["lqr_weights"]
    st_names = [s["name"] for s in res["stages"]]
    assert any("Rising/Falling PID" in n for n in st_names)
    assert any("Cruising/Turn LQR" in n for n in st_names)

    # Phase costs helper agrees with additive total
    pc = phase_costs(best["metrics"], DEFAULT_PARAMS["flying_height"])
    assert pc["total"] == pytest.approx(best["cost"])

    # 5. sim_server persists BOTH tuned PID gains and LQR weights when target='auto'
    state_dir = tmp_path / "local_auto"
    state_dir.mkdir(parents=True)
    monkeypatch.setattr(sim_server, "STATE_DIR", state_dir)
    monkeypatch.setattr(sim_server, "OVERRIDES", state_dir / "overrides.json")
    monkeypatch.setattr(sim_server, "TUNE_PROGRESS", prog_path)
    monkeypatch.setattr(sim_server, "tune_proc", None)
    monkeypatch.setattr(sim_server, "_last_tune_data", {"status": "idle"})
    monkeypatch.setattr(sim_server, "_auto_saved_sig", None)
    st = sim_server.tune_status("model")
    assert st["auto_saved"] is True
    assert st["stored"]["has_tuned_pid"] is True
    assert st["stored"]["has_tuned_lqr"] is True
    saved = json.loads((state_dir / "overrides.json").read_text(encoding="utf-8"))
    assert "kp_heave" in saved["model"] and "lqr_weights" in saved["model"]


def test_null_collapsing_physics_and_hud_anticlipping():
    from hydrofoil.actuation.hinge import thin_airfoil_coeffs
    from hydrofoil.estimation.kalman import HeaveFilter
    from hydrofoil.forces.foil import Foil, FoilGeometry, FoilTables, main_flap_effectiveness
    from hydrofoil.forces.hull import HullHydrostatics
    from hydrofoil.monitor import FlightMonitor
    from hydrofoil.viewer import Hud

    # 1. Collapsing foil dimensions, zero speed, and extreme flap fractions stay finite & reset stale CL
    assert main_flap_effectiveness(0.0, 1.0) == 0.0
    assert main_flap_effectiveness(1.2, 0.5) > 0.0
    assert thin_airfoil_coeffs(0.0)["ch_delta"] == 0.0
    assert np.isfinite(thin_airfoil_coeffs(1.2)["tau"])

    polar = FoilTables.naca0009()
    zero_foil = Foil(FoilGeometry(chord=0.0, span=0.0), polar, np.array([0.0, 0.0, 0.5]))
    assert zero_foil.aspect_ratio == 0.0 and zero_foil.lift_slope_factor == 0.0
    assert np.allclose(zero_foil.generalized_force(np.array([5.0, 0.0, 0.0, 0.0, 0.0, 0.0])), 0.0)
    with pytest.raises(ValueError):
        Foil(FoilGeometry(chord=0.2, span=0.5), polar, np.zeros(3), normal_axis=np.zeros(3))

    vp = VehiclePhysics(dict(MANNED_PARAMS, controller="pid", sensors="ideal"))
    _ = vp.left_foil.generalized_force(np.array([8.0, 0.0, 0.0, 0.0, 0.0, 0.0]), delta=5.0, submergence_over_chord=3.0)
    assert vp.left_foil.last_CL != 0.0
    assert np.allclose(vp.left_foil.generalized_force(np.zeros(6)), 0.0)
    assert vp.left_foil.last_CL == 0.0

    # 2. Collapsing hull, HeaveFilter w0=0, and FlightMonitor duplicate timestamps
    hull_zero = HullHydrostatics(0.0, 0.0, 0.0, -0.35, 0.0, nx=0, ny=0)
    assert np.all(np.isfinite(hull_zero.buoyancy(np.zeros(6), 0.0)))
    assert np.all(np.isfinite(hull_zero.drag(np.zeros(6), 0.0, 120.0, 6000.0)))

    hf = HeaveFilter(0.01, wave={"w0": 0.0, "sigma": 0.0})
    hf.init(0.5)
    assert np.all(np.isfinite(hf.step(0.0, 0.5, True)))

    mon = FlightMonitor(dict(MANNED_PARAMS))
    s_fly = np.zeros(12); s_fly[6] = 8.0; s_fly[2] = MANNED_PARAMS["hull_offset"] - MANNED_PARAMS["flying_height"]
    cmds = {"delta_main": 0.0, "delta_elev": 0.0, "delta_roll": 0.0}
    for _ in range(12):
        mon.buf["t"].append(1.0); mon.buf["dz"].append(0.0); mon.buf["roll"].append(0.0)
        mon.buf["vz"].append(0.0); mon.buf["lim"].append(False)
    mon.air_since = 0.0; mon.established = True
    assert isinstance(mon.update(1.05, vp, s_fly, cmds, CalmWater(), None, True), list)

    # 3. Design guards handle None values, missing payload fields, and collapsed sliders cleanly
    bad_edit = apply_design(dict(MANNED_PARAMS), {
        "main_chord": 0.0, "main_span": -2.0, "elev_chord": None, "hull_length": 0.0, "strut_length": 0.0,
        "payloads": [{"name": "Rider", "mass": None, "x": None, "y": 0.0, "z": None}],
    })
    assert bad_edit["main_chord"] >= 0.02 and bad_edit["main_span"] >= 0.05 and bad_edit["hull_length"] >= 0.2
    d_bad = derived(bad_edit)
    assert np.isfinite(d_bad["wing_area"]) and np.isfinite(d_bad["wing_loading"])

    # 4. Narrow 640px MuJoCo HUD renders long status/alert labels without clipping or crashing
    from PIL import Image, ImageDraw
    hud = Hud()
    im = Image.new("RGB", (640, 110))
    g = ImageDraw.Draw(im)
    short, fnt = hud._fit_text(g, "X" * 120, "ui", 16, 100)
    assert g.textlength(short, font=fnt) <= 100 + 1e-6 and short.endswith("…")
    arr = hud.render(640, 110, {
        "time": "123.4 s", "mode": "Emergency_landing",
        "ctl": "Auto (PID (waiting for height settle)), noisy sensors + Kalman [sup]",
        "roll": 12.4, "pitch": -3.1, "bank_ref": 10.0,
        "alert": ("danger", "Very long multi-fault controller warning that would otherwise clip past the right edge of a small viewer window"),
        "keys": "Up/Down height  Left/Right bank",
        "cells": {1: ("Speed", "18.42", "km/h", "10.0 kn, target 18.00 km/h", False),
                  2: ("Ride height", "0.52", "m", "target 0.50 m", False),
                  4: ("Thrust", "995", "N", "at limit: left,right,elev", True)},
    })
    assert arr.shape == (110, 640, 3) and arr.dtype == np.uint8
