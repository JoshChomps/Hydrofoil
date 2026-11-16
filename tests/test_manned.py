"""Manned preset (MANNED_PARAMS, 250-300 kg): sizing, takeoff and rider CG (docs/19, "Manned sizing")."""
import numpy as np
import pytest

from hydrofoil.linearize import trim
from hydrofoil.params import MANNED_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import CalmWater


@pytest.mark.parametrize("mass", [250.0, 300.0])
def test_trim_flap_keeps_authority_from_takeoff_to_cruise(mass):
    vp = VehiclePhysics(dict(MANNED_PARAMS, mass=mass, controller="pid"))
    flap = {sp: trim(vp, sp)[1][0] for sp in (6.0, 8.0, 9.5)}
    assert all(abs(f) < 12.0 for f in flap.values())          # of +-20: margin both ways for waves and control
    assert flap[6.0] > flap[8.0] > flap[9.5]                  # less flap as q rises
    assert trim(vp, 8.0)[1][3] < 250.0                        # cruise thrust [N]


def _fly(T, cg=(0.0, 0.0), shift_at=None, shift=None, **extra):
    p = dict(MANNED_PARAMS, cg_offset=cg, **extra)
    vp = VehiclePhysics(p)
    zref = p["hull_offset"] - p["flying_height"]
    s, t, rec = vp.rest_state(), 0.0, []
    for i in range(int(T / p["dt"])):
        if shift_at is not None and i == int(shift_at / p["dt"]):
            vp.set_cg_offset(shift)
        s, c, th, d, _ = vp.step(s, t, CalmWater()); t += p["dt"]
        rec.append((t, s[2] - zref, np.degrees(s[3]), d))
    return vp, np.array(rec)


def test_300kg_with_rider_off_centre_takes_off_and_holds_height():
    """10 cm lateral CG at rest: with the 10 N m drive cap the low-side flap stalled short and the boat rolled over."""
    vp, r = _fly(15.0, cg=(0.0, 0.10), mass=300.0)
    assert vp.lqr.engaged
    after = r[:, 0] > 8.0
    assert r[after, 3].max() == 0.0                           # foilborne
    assert np.abs(r[after, 1]).max() < 0.02 and np.abs(r[after, 2]).max() < 0.5


def test_rider_moving_in_flight_is_absorbed():
    vp, r = _fly(25.0, mass=300.0, shift_at=12.0, shift=(0.2, -0.15))
    after = r[:, 0] > 12.0
    assert r[after, 3].max() == 0.0
    assert np.abs(r[after, 1]).max() < 0.05 and np.abs(r[after, 2]).max() < 2.0
    assert abs(r[-1, 1]) < 0.005
