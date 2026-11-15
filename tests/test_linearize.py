import numpy as np
import pytest

from hydrofoil.linearize import trim, linearize, reduced, REDUCED_NAMES, _rhs
from hydrofoil.params import DEFAULT_PARAMS
from hydrofoil.vehicle import VehiclePhysics


@pytest.fixture(scope="module")
def plant():
    vp = VehiclePhysics(dict(DEFAULT_PARAMS))
    x, u = trim(vp, 4.0)
    A, B = linearize(vp, x, u)
    return vp, x, u, A, B


def test_trim_is_an_equilibrium_and_flap_grows_as_speed_drops(plant):
    vp, x, u, _, _ = plant
    assert np.max(np.abs(_rhs(vp, x, u)[[6, 8, 10]])) < 1e-8
    assert -5.0 < u[0] < 5.0 and abs(u[1]) < 3.0 and 0.0 < u[3] < 50.0   # incidence carries the weight at cruise
    _, u6 = trim(vp, 6.0)
    assert u6[0] < u[0]                      # lift ~ u^2: less flap needed when faster


def test_open_loop_poles_have_expected_structure(plant):
    _, _, _, A, B = plant
    Ar, Br, _ = reduced(A, B)
    ev = np.linalg.eigvals(Ar)
    assert np.sum(ev.real > 0) == 1          # exactly one unstable (lateral spiral-type) mode
    assert 0.1 < ev.real.max() < 0.5
    ctrb = np.hstack([np.linalg.matrix_power(Ar, k) @ Br for k in range(Ar.shape[0])])
    assert np.linalg.matrix_rank(ctrb, tol=1e-8) == Ar.shape[0]


def test_linear_model_predicts_a_small_step(plant):
    vp, x, u, A, B = plant
    du = np.array([0.1, 0.0, 0.0, 0.0])
    lin = (A @ np.zeros(12) + B @ du)
    nl = _rhs(vp, x, u + du) - _rhs(vp, x, u)
    assert np.allclose(lin[[6, 8, 10]], nl[[6, 8, 10]], rtol=0.02, atol=1e-5)
