"""Flight-mode supervisor (control/supervisor.py) on the manned boat: protocol, envelope and fault responses."""
import numpy as np

from hydrofoil.params import MANNED_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import CalmWater


def _mission(plan, T, fault=None, **extra):
    p = dict(MANNED_PARAMS, supervisor=True, **extra)
    vp = VehiclePhysics(p)
    sup, dt = vp.supervisor, p["dt"]
    s, t, rec, wet_prev, touch = vp.rest_state(), 0.0, [], True, []
    for i in range(int(T / dt)):
        for tt, cmd in plan:
            if i == int(round(tt / dt)):
                sup.pilot.update(cmd)
        if fault is not None and i == int(round(fault[0] / dt)):
            fault[1](vp)
        s, c, th, dr, _ = vp.step(s, t, CalmWater()); t += dt
        vz = -s[6] * np.sin(s[4]) + s[8] * np.cos(s[4]) * np.cos(s[3])
        wet = dr > 1e-4
        if wet and not wet_prev:
            touch.append((t, s[6], vz, np.degrees(s[3]), np.degrees(s[4])))
        wet_prev = wet
        rec.append((t, sup.mode, s[6], p["hull_offset"] - s[2], np.degrees(s[3]), np.degrees(s[4])))
    return vp, rec, touch


CYCLE = [(0.0, dict(speed=3.0)), (3.0, dict(foil=True, speed=7.0)), (30.0, dict(foil=False, speed=3.0))]


def test_full_cycle_takes_off_cruises_and_lands_gently():
    vp, rec, touch = _mission(CYCLE, 42.0)
    modes = [to for _t, _from, to, _why in vp.supervisor.log]
    assert modes == ["TAKEOFF", "FOILING", "LANDING", "DISPLACEMENT"]
    foiling = [r for r in rec if r[1] == "FOILING"]
    assert all(abs(r[3] - 0.25) < 0.04 for r in foiling[200:])           # holds the 0.25 m hull clearance
    assert len(touch) == 1 and 0.0 < touch[0][2] < 0.3                   # one touchdown, sink < 0.3 m/s
    assert abs(touch[0][3]) < 2.0 and abs(touch[0][4]) < 3.0             # level
    assert vp.supervisor.mode == "DISPLACEMENT" and not vp.supervisor.faults


def test_envelope_clamps_pilot_speed_and_height():
    plan = CYCLE[:2] + [(22.0, dict(speed=3.0, height=0.0))]
    vp, rec, _ = _mission(plan, 30.0)
    sup = vp.supervisor
    assert sup.mode == "FOILING"
    assert abs(sup.speed_ref - sup.c["v_min_foil"]) < 1e-9               # no stall below the speed floor
    assert abs(sup.height_ref - sup.c["h_min"]) < 1e-6                   # no lower than the minimum clearance
    assert rec[-1][2] > sup.c["v_min_foil"] - 0.4


def _jam(angle):
    def f(vp):
        d = vp.drives["right"]
        d.update = lambda cmd, flow, a=angle: a
        d.delta, d.rate = np.radians(angle), 0.0
        d.telemetry = dict(d.telemetry, limited=True)                    # a jammed drive stalls at its limit
    return f


def test_flap_jammed_off_trim_lands_level_instead_of_capsizing():
    """Without the supervisor a flap jammed at +8 deg rolled the boat past 50 deg (docs/19, failure table)."""
    vp, rec, touch = _mission(CYCLE[:2], 32.0, fault=(20.0, _jam(8.0)))
    sup = vp.supervisor
    assert "flap_right" in sup.faults and sup.mode == "DISPLACEMENT"
    assert max(abs(r[4]) for r in rec) < 10.0
    assert touch and touch[0][2] < 0.3
    sup.pilot["foil"] = True                                             # a latched fault blocks a new takeoff
    vp.step(vp.rest_state(), 40.0, CalmWater())
    assert sup.mode == "DISPLACEMENT"


def test_elevator_power_loss_cuts_thrust_and_limits_the_nose_down():
    """Unsupervised: pitch -13 deg and 1 m/s sink at 8 m/s within about 1 s."""
    def limp(vp):
        vp.drives["elev"].kp = 0.0; vp.drives["elev"].kd = 0.0
    vp, rec, touch = _mission(CYCLE[:2], 28.0, fault=(20.0, limp))
    assert "elev_loss" in vp.supervisor.faults
    assert min(r[5] for r in rec if r[0] > 20.0) > -8.0
    assert touch and touch[0][2] < 0.8


def test_calm_takeoff_does_not_fabricate_wave_amplitude():
    plan = [(0.0, dict(speed=3.0)), (3.0, dict(foil=True, speed=7.0, height=0.20))]
    vp, rec, _ = _mission(plan, 25.0, sensors="noisy")
    assert vp.supervisor.mode == "FOILING"
    assert vp.supervisor.wave_amp < 0.04
    assert abs(rec[-1][3] - 0.20) < 0.03


def test_frozen_fresh_height_sensor_triggers_height_lost_and_lands():
    def freeze(vp):
        ch = vp.sensors.channels["height"]
        val = ch.value.copy()
        orig = ch.step
        def stuck(st, t, dt, rng):
            orig(st, t, dt, rng)
            ch.value = val
            return val
        ch.step = stuck
    plan = [(0.0, dict(speed=3.0)), (3.0, dict(foil=True, speed=8.0)), (21.0, dict(height=0.50))]
    vp, rec, _ = _mission(plan, 35.0, fault=(20.0, freeze), sensors="noisy")
    assert "height_lost" in vp.supervisor.faults
    assert vp.supervisor.mode == "DISPLACEMENT"
    assert max(abs(r[4]) for r in rec) < 5.0 and max(abs(r[5]) for r in rec) < 5.0


def test_limp_main_flap_is_isolated_and_lands_level():
    plan = [(0.0, dict(speed=3.0)), (3.0, dict(foil=True, speed=8.0)), (18.0, dict(bank=8.0))]
    vp, rec, _ = _mission(plan, 30.0, fault=(20.0, lambda v: setattr(v.drives["left"], "cap", 1e-6)))
    assert "flap_left" in vp.supervisor.faults
    assert vp.supervisor.fault_info["left"]["mode"] == "limp"
    assert vp.supervisor.mode == "DISPLACEMENT"
    assert max(abs(r[4]) for r in rec) < 12.0
