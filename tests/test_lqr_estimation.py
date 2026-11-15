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


def test_lqr_tolerates_more_loop_delay_than_it_was_designed_for():
    """Gains designed for the nominal drive (10 ms); the real drive is then given 80 ms of command delay.
    The earlier design (no drive or delay in the model) limit-cycled from 30 ms: about 2 deg rms of flap in
    calm water. Cruise must stay quiet."""
    p = dict(DEFAULT_PARAMS, controller="lqr")
    vp = VehiclePhysics(p)
    for d in vp.drives.values():
        d.delay_steps = 8; d.reset()
    s, t, flap, z = vp.rest_state(), 0.0, [], []
    for _ in range(int(30.0 / p["dt"])):
        s, c, *_ = vp.step(s, t, CalmWater()); t += p["dt"]
        flap.append(c["delta_main"]); z.append(s[2])
    assert vp.lqr.engaged
    assert np.std(flap[-800:]) < 0.2 and np.std(z[-800:]) < 0.002
    assert abs(z[-1] - (p["hull_offset"] - p["flying_height"])) < 0.02


def test_delay_aware_design_is_stable_on_its_own_model_at_every_scheduled_speed():
    from hydrofoil.control.lqr import design_delay_aware, augmented_model
    vp = VehiclePhysics(dict(DEFAULT_PARAMS, controller="pid"))
    for speed in (3.0, 5.0, 7.0):
        for d in (1, 5):
            K, _ = design_delay_aware(vp, speed, 0.01, d)
            Ad, Bd, _ = augmented_model(vp, speed, 0.01, d)
            assert np.abs(np.linalg.eigvals(Ad - Bd @ K)).max() < 0.999


def test_incidence_leaves_the_trim_flap_near_neutral_with_authority_both_ways():
    vp = VehiclePhysics(dict(DEFAULT_PARAMS, controller="pid"))
    assert abs(trim(vp, 4.0)[1][0]) < 5.0                    # cruise: was 14 deg of a 20 deg limit at zero incidence
    for speed in (3.0, 7.0):
        assert abs(trim(vp, speed)[1][0]) < 14.0             # both ends of the schedule keep >= 6 deg of margin


def test_height_sensor_reads_the_local_surface_and_flags_fresh_samples():
    suite = SensorSuite(seed=2, scale=0.0)                    # noise off: exact values
    x = np.zeros(12); x[2] = 0.05
    fresh_z = 0
    for k in range(100):
        m = suite.measure(x, k * 0.01, 0.01, surface_elev=0.08)
        fresh_z += "z" in suite.fresh
    assert abs(m["z"] - 0.13) < 1e-9                          # depth relative to the raised surface, not still water
    assert 17 <= fresh_z <= 21                                # 20 Hz sensor in a 100 Hz loop: new on 1 step in 5
    assert abs(m["az"]) < 1e-9                                # steady flight: no vertical acceleration


def test_heave_filter_rejects_waves_keeps_real_height_and_survives_accelerometer_bias():
    from hydrofoil.estimation.kalman import HeaveFilter
    dt, rng = 0.01, np.random.default_rng(0)

    def run(wave_amp, wave_freq):
        f = HeaveFilter(dt); f.init(0.05)
        hist, err = [], []
        for k in range(6000):
            t = k * dt
            z = 0.05 + 0.02 * np.sin(0.5 * t) + (0.03 if t > 40 else 0.0)      # slow real motion + a real step
            az = -0.005 * np.sin(0.5 * t)
            hist.append(z + wave_amp * np.cos(wave_freq * t) + 0.01 * rng.standard_normal())
            f.step(az + 0.02 + 0.05 * rng.standard_normal(), hist[max(0, k - 5)], k % 5 == 0)   # biased accel, late range
            if 2000 < k < 4000 or k > 5000:
                err.append(f.z - z)
        return np.std(err), abs(np.mean(err))

    sd, mean = run(0.0, 1.0)
    assert sd < 0.008 and mean < 0.004                        # calm: tracks the motion and the step
    sd, mean = run(0.10, 8.5)                                 # 10 cm wave at a head-sea encounter frequency
    assert sd < 0.25 * 0.0707 and mean < 0.005                # under a quarter of the wave's 71 mm rms gets through


def test_rider_shifting_weight_mid_flight_is_absorbed():
    """LQR designed for the nominal layout; the CG then moves 10 cm aft and 5 cm to the side in flight."""
    p = dict(DEFAULT_PARAMS, controller="lqr")
    vp = VehiclePhysics(p)
    zref = p["hull_offset"] - p["flying_height"]
    s, t, z, phi, draft = vp.rest_state(), 0.0, [], [], []
    for i in range(int(40.0 / p["dt"])):
        if i == 2000:
            vp.set_cg_offset((-0.10, 0.05))
        s, c, th, d, _ = vp.step(s, t, CalmWater()); t += p["dt"]
        z.append(s[2] - zref); phi.append(np.degrees(s[3])); draft.append(d)
    assert max(draft[2000:]) == 0.0                           # never touches down
    assert max(abs(v) for v in z[2000:]) < 0.03 and max(abs(v) for v in phi[2000:]) < 2.0
    assert abs(z[-1]) < 0.005 and abs(phi[-1]) < 0.2          # integrators re-trim it
    assert abs(c["delta_roll"]) > 0.5                         # ...by holding differential flap against the offset
