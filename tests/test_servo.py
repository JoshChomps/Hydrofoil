"""Closed-loop surface drives (actuation/servo.py) and their wiring into VehiclePhysics."""

import numpy as np

from hydrofoil.params import DEFAULT_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.actuation.loads import _Flow
from hydrofoil.waves.airy import CalmWater


def _vp(**over):
    p = dict(DEFAULT_PARAMS)
    p.update(over)
    return VehiclePhysics(p)


def _flow(vp, name, V):
    foil = vp.left_foil if name != "elev" else vp.elev_foil
    return _Flow(vp.hinges[name], foil, V, np.radians(1.0), foil.lift_slope_factor * 0.85)


def _step(vp, name, V, cmd=10.0, n=60):
    d = vp.drives[name]
    d.reset()
    f = _flow(vp, name, V)
    out = [d.update(cmd, f) for _ in range(n)]
    return np.array(out), d


def test_servo_tracks_step_and_respects_torque_cap():
    vp = _vp()
    ang, d = _step(vp, "left", 4.0)
    assert abs(ang[-1] - 10.0) < 1.0                  # small droop from the hinge moment (outer loop integrates it)
    cap_current = d.cap / d.spec.gear.ratio / d.spec.motor.kt
    assert d.telemetry["I"] <= cap_current + 1e-6


def test_lower_torque_cap_slows_the_surface():
    fast, _ = _step(_vp(), "left", 6.0)
    slow, _ = _step(_vp(servo_params={"torque_cap": 2.5}), "left", 6.0)
    t90 = lambda a: int(np.argmax(a >= 0.9 * a[-1]))
    assert t90(slow) > t90(fast)


def test_back_emf_limits_rate_at_low_bus_voltage():
    _, d_hi = _step(_vp(), "elev", 4.0, cmd=15.0, n=3)
    _, d_lo = _step(_vp(v_bus=0.5), "elev", 4.0, cmd=15.0, n=3)
    assert abs(d_lo.rate) < abs(d_hi.rate)


def test_hydrodynamic_load_droops_flap_more_at_high_speed():
    a4, _ = _step(_vp(), "left", 4.0)
    a8, _ = _step(_vp(), "left", 8.0)
    assert a8[-1] < a4[-1] < 10.0


def test_mixer_limits_are_per_physical_surface():
    vp = _vp(allocator="legacy")
    raw = {"delta_main": 20.0, "delta_roll": 20.0, "delta_elev": 0.0}
    st = vp.rest_state(); st[6] = 4.0
    env = vp.calm_sample()
    for _ in range(100):
        c = vp._servo_commands(raw, st, env)
    right = c["delta_main"] + c["delta_roll"]
    assert right <= DEFAULT_PARAMS["flap_stop_deg"] + 1e-6
    assert c["target"]["right"] == 40.0


def test_limp_drives_lose_lift_and_boat_settles_on_hull():
    vp = _vp()
    st, t = vp.rest_state(), 0.0
    for _ in range(2500):                               # take off and settle (25 s)
        st, c, _, draft, _ = vp.step(st, t, CalmWater()); t += 0.01
    assert draft <= 1e-6
    for d in vp.drives.values():                        # drive fault: no torque
        d.cap = 1e-6
    for _ in range(1000):
        st, c, _, draft, _ = vp.step(st, t, CalmWater()); t += 0.01
    assert c["delta_main"] < 5.0                        # flaps trailed back toward the stream
    assert draft > 0.0                                  # hull back in the water
