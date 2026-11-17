"""Backlash, gear compliance, flap twist, command slew limit and the parameter co-dependency checker (docs/20)."""

import numpy as np
import pytest


class _Flow:
    """Frozen linear hinge load: TE-up offset plus restoring stiffness."""
    V = 8.0

    def __init__(self, m0=-4.0, k=100.0):
        self.m0, self.k = m0, k

    def __call__(self, d):
        return self.m0 - self.k * d


def _drive(**kw):
    from hydrofoil.params import MANNED_PARAMS
    from hydrofoil.actuation.loads import surface_models
    from hydrofoil.actuation.servo import ServoDrive, DEFAULT_SERVO_PARAMS
    from hydrofoil.actuation.drive import DriveSpec
    p = dict(MANNED_PARAMS)
    sp = dict(DEFAULT_SERVO_PARAMS, torque_cap=28.0)
    sp.update(kw)
    return ServoDrive(surface_models(p)["left"], DriveSpec(), 0.01, 30.0, **sp)


def _settle(d, cmd=5.0, T=1.5, flow=None):
    f = flow or _Flow()
    a = None
    for _ in range(int(T / 0.01)):
        a = d.update(cmd, f)
    return a


def test_stiff_compliant_model_matches_the_rigid_one():
    a_rigid = _settle(_drive())
    a_stiff = _settle(_drive(gear_stiffness=1e7))
    assert abs(a_rigid - a_stiff) < 1e-3


def test_backlash_offsets_the_surface_unless_the_encoder_is_on_it():
    b = 0.3
    ref = _settle(_drive())
    d = _drive(backlash_deg=b, encoder="motor")
    a = _settle(d)
    # the load pushes the trailing edge up: the surface rests on the lower flank, b/2 below the motor side
    assert abs((np.degrees(d.th_m) - a) - b / 2) < 1e-3
    # equilibrium eta kp (cmd - theta_m) = load(theta_s), theta_s = theta_m - b/2: the surface sits
    # (b/2) eta kp / (eta kp + k_load) below where the rigid drive put it
    ekp = 0.85 * 250.0
    assert abs(a - (ref - b / 2 * ekp / (ekp + _Flow().k))) < 1e-3
    d = _drive(backlash_deg=b, encoder="output")
    assert abs(_settle(d) - ref) < 1e-3                  # an output encoder takes the free play out


def test_gear_compliance_adds_load_over_stiffness():
    kg = 5000.0
    d = _drive(gear_stiffness=kg, encoder="motor")
    a = _settle(d)
    load = abs(_Flow()(np.radians(a)))
    assert abs((np.degrees(d.th_m) - a) - np.degrees(load / kg)) < 2e-3


def test_flap_twist_is_load_over_twist_stiffness():
    k_tw = 3 * 250.0 / 1.1
    d = _drive(twist_k=k_tw)
    a = _settle(d)
    m = _Flow()(np.radians(a))
    assert abs((a - np.degrees(d.th_s)) - np.degrees(m / k_tw)) < 1e-6
    assert a < np.degrees(d.th_s)                        # TE-up load: the span-mean flap sits below the root


def test_command_slew_limit_caps_the_takeoff_step():
    from hydrofoil.params import MANNED_PARAMS
    from hydrofoil.vehicle import VehiclePhysics
    from hydrofoil.waves.airy import CalmWater
    p = dict(MANNED_PARAMS, controller="pid", sensors="ideal")
    vp = VehiclePhysics(p)
    s, t, prev, worst = vp.rest_state(), 0.0, 0.0, 0.0          # surfaces start at 0: the first step counts too
    for _ in range(300):
        s, c, *_ = vp.step(s, t, CalmWater()); t += p["dt"]
        tg = c["target"]["left"]
        worst = max(worst, abs(tg - prev) / p["dt"])
        prev = tg
    assert worst <= p["cmd_slew_deg_s"] + 1e-6


@pytest.mark.parametrize("preset", ["model", "manned"])
def test_no_parameter_co_dependency_fails(preset):
    from hydrofoil.params import DEFAULT_PARAMS, MANNED_PARAMS
    from hydrofoil.consistency import check
    rows = check(DEFAULT_PARAMS if preset == "model" else MANNED_PARAMS)
    fails = [r for r in rows if r["status"] == "FAIL"]
    assert not fails, fails


def test_checker_catches_a_stale_trim_feedforward_and_an_overstressed_shaft():
    from hydrofoil.params import MANNED_PARAMS
    from hydrofoil.consistency import check
    rows = {r["check"]: r for r in check(dict(MANNED_PARAMS, trim_ff_deg=MANNED_PARAMS["trim_ff_deg"] + 2.0))}
    assert rows["trim_ff_deg = trim flap at trim_ff_speed"]["status"] == "FAIL"
    rows = {r["check"]: r for r in check(dict(MANNED_PARAMS, main_flap_frac=0.4, main_flap_span_frac=0.5,
                                                main_flap_setback=0.0))}
    assert rows["trim_ff_deg = trim flap at trim_ff_speed"]["status"] == "FAIL"   # geometry changed, FF not
    assert rows["main_surface_inertia matches the flap geometry (+-30 %)"]["status"] == "WARN"
