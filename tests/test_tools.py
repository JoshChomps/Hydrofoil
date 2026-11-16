"""Design layer, flight observer, scoring and classical tuning (docs/19, "Boat editor", "Flight observer", "Autotuner")."""
import numpy as np
import pytest

from hydrofoil.design import apply_design, derived, mass_properties
from hydrofoil.monitor import FlightMonitor
from hydrofoil.params import MANNED_PARAMS
from hydrofoil.tuning.classical import RULES, bryson_update
from hydrofoil.tuning.scoring import cost, run_scenario
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import CalmWater


def test_default_payloads_reproduce_the_preset_mass_and_cg():
    m = mass_properties(MANNED_PARAMS, None)
    assert m["mass"] == pytest.approx(275.0) and np.allclose(m["cg"], 0.0)
    assert np.allclose(m["inertia"], [MANNED_PARAMS["ixx"], MANNED_PARAMS["iyy"], MANNED_PARAMS["izz"]])


def test_moving_the_rider_aft_moves_the_cg_aft_and_keeps_the_mass():
    p = apply_design(dict(MANNED_PARAMS), {"payloads": [{"name": "Rider", "mass": 75.0, "x": -0.83, "y": 0.0, "z": 0.33}]})
    assert p["mass"] == pytest.approx(275.0)
    assert p["cg_offset"][0] == pytest.approx(-0.4 * 75 / 275, rel=1e-6)
    assert p["iyy"] > MANNED_PARAMS["iyy"]                                  # weight further from the CG


def test_smaller_foil_raises_wing_loading_and_takeoff_speed():
    base = derived(dict(MANNED_PARAMS))
    small = derived(apply_design(dict(MANNED_PARAMS), {"main_chord": 0.22}))
    assert small["wing_loading"] > base["wing_loading"]
    assert small["v_min_foil"] >= base["v_min_foil"]
    assert base["cruise_flap"] == pytest.approx(-2.78, abs=0.2)          # matches the trim checked in docs/19


def test_observer_is_quiet_in_calm_cruise_and_flags_a_dead_elevator():
    def fly(fail):
        p = dict(MANNED_PARAMS)
        vp, mon = VehiclePhysics(p), FlightMonitor(p)
        s, t = vp.rest_state(), 0.0
        for _ in range(int(18.0 / p["dt"])):
            if fail and abs(t - 15.0) < 0.005:
                vp.drives["elev"].kp = vp.drives["elev"].kd = 0.0
            s, c, th, d, _ = vp.step(s, t, CalmWater()); t += p["dt"]
            mon.update(t, vp, s, c, CalmWater(), c.get("drive"), d <= 1e-6)
        return mon
    calm = fly(False)
    assert not [e for e in calm.log if e["t"] > 6.0 and e["level"] != "ok"]   # nothing after takeoff settles
    dead = fly(True)
    assert any(e["level"] == "danger" and e["t"] > 15.0 for e in dead.log)


def test_scoring_runs_and_penalises_failure():
    m = run_scenario(dict(MANNED_PARAMS, controller="pid"), "head")
    assert not m["failed"] and m["z_rms"] < 0.1
    assert cost({"head": m}, 0.5) < cost({"head": {"failed": True}}, 0.5)


def test_tuning_rules_and_bryson_step():
    assert RULES["Ziegler-Nichols"] == (0.60, 0.50, 0.125)
    mult = dict.fromkeys(["z", "theta", "phi", "w", "q", "iz", "du"], 1.0)
    over = {"head": {"failed": False, "z_rms": 0.05, "pitch_peak": 0.5, "roll_peak": 0.2, "flap_rate_rms": 20.0, "limited_frac": 0.0}}
    new = bryson_update(mult, over, 0.5)                                   # height overshot its 15 mm target: tighten
    assert new["z"] < 1.0 and new["theta"] > 1.0 and new["du"] > 1.0
