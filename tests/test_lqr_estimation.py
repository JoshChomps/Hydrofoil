import numpy as np
import pytest

from hydrofoil.control.lqr import design, STATE_IDX
from hydrofoil.linearize import trim, linearize, reduced
from hydrofoil.params import DEFAULT_PARAMS
from hydrofoil.sensors.models import SensorSuite, Channel
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import CalmWater


def _fly(T, **extra):
    p = dict(DEFAULT_PARAMS); p.update(extra)
    vp = VehiclePhysics(p)
    s, t, rec = vp.rest_state(), 0.0, []
    for _ in range(int(T / p["dt"])):
        s, c, th, d, _ = vp.step(s, t, CalmWater()); t += p["dt"]; rec.append((t, s.copy(), d))
    return p, vp, s, rec


def test_lqr_gain_stabilises_the_linear_plant_including_the_unstable_mode():
    vp = VehiclePhysics(dict(DEFAULT_PARAMS))
    K, u0 = design(vp, 4.0)
    x0, ut = trim(vp, 4.0)
    A, B = linearize(vp, x0, ut)
    Ar, Br, _ = reduced(A, B)
    n = len(STATE_IDX)
    As, Bs = Ar[np.ix_(STATE_IDX, STATE_IDX)], Br[STATE_IDX, :]
    Aa = np.zeros((n + 3, n + 3)); Aa[:n, :n] = As
    for i in range(3):
        Aa[n + i, i] = 1.0
    Ba = np.vstack([Bs, np.zeros((3, 3))])
    assert np.max(np.linalg.eigvals(Aa - Ba @ K).real) < -0.3


def test_lqr_takes_over_after_takeoff_without_touchdown_and_holds_height():
    p, vp, s, rec = _fly(20.0, controller="lqr")
    assert vp.lqr.engaged
    zref = p["hull_offset"] - p["flying_height"]
    assert abs(s[2] - zref) < 0.02 and abs(np.degrees(s[3])) < 0.2
    ft = next(t for t, _, d in rec if d <= 1e-6)
    assert max(d for t, _, d in rec if t > ft + 1.0) == 0.0


def test_sensor_channel_noise_rate_latency_and_dropout():
    rng = np.random.default_rng(1)
    ch = Channel(("z",), (2,), 20.0, 0.01, latency_s=0.05, dropout=0.0)
    x = np.zeros(12); x[2] = 1.0
    vals, firsts = [], None
    for k in range(2000):
        v = ch.step(x, k * 0.01, 0.01, rng)
        if v is not None and firsts is None:
            firsts = k * 0.01
        if v is not None:
            vals.append(v[0])
    assert abs(firsts - 0.05) < 0.011                      # latency
    assert abs(np.std(vals) - 0.01) < 0.003 and abs(np.mean(vals) - 1.0) < 0.003
    drop = Channel(("z",), (2,), 100.0, 0.0, dropout=0.5)
    got = [drop.step(x, k * 0.01, 0.01, rng) for k in range(2000)]
    fresh = sum(1 for a, b in zip(got, got[1:]) if a is not b)     # a new array object = a new accepted sample
    assert 800 < fresh < 1200                                       # ~half of the 2000 samples survive 50 % dropout
    assert SensorSuite(seed=3).measure(x, 0.0, 0.01).keys() >= {"p", "phi", "theta", "u"}


def test_noisy_sensors_with_kalman_estimator_still_fly():
    p, vp, s, rec = _fly(20.0, controller="lqr", sensors="noisy")
    zref = p["hull_offset"] - p["flying_height"]
    assert abs(s[2] - zref) < 0.04 and abs(np.degrees(s[3])) < 0.5
    ft = next(t for t, _, d in rec if d <= 1e-6)
    assert max(d for t, _, d in rec if t > ft + 1.0) == 0.0
