"""
Regression tests for the method / unit / math fixes made on 2026-11-04
(see docs/13-math-audit-and-pid-critique.md). Each test pins one thing that was wrong.
"""

import numpy as np
import pytest

from hydrofoil.constants import G
from hydrofoil.control.pid import PID, HydrofoilController
from hydrofoil.control.surge import SurgeController
from hydrofoil.dynamics.rigid_body import RigidBody
from hydrofoil.forces.foil import Foil, FoilGeometry, FoilTables
from hydrofoil.forces.hull import hull_buoyancy_force, hull_draft
from hydrofoil.params import DEFAULT_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import AiryWave, CalmWater

POLAR = FoilTables.naca0009()
FAST_ACT = {"rate_limit_deg_s": 1e6, "tau": 1e-6, "delay_steps": 0}
GAINS_1 = {k: {"Kp": 1.0, "Ki": 0.0, "Kd": 0.0} for k in ("heave", "pitch", "roll")}


# --- PID / controller units -------------------------------------------------

def test_roll_pitch_gains_are_degrees_per_degree():
    """Angle errors arrive in radians; the PID must see degrees (Kp=1 -> 5 deg error = 5 deg flap)."""
    ctrl = HydrofoilController(GAINS_1, dt=0.01, actuator_params=FAST_ACT)
    cmds = ctrl.update(0.0, 0.0, 0.0, np.radians(5.0), 0.0, np.radians(-3.0))
    assert cmds["delta_roll"] == pytest.approx(5.0, abs=0.01)
    assert cmds["delta_elev"] == pytest.approx(-3.0, abs=0.01)


def test_heave_gain_is_degrees_per_metre():
    ctrl = HydrofoilController(GAINS_1, dt=0.01, actuator_params=FAST_ACT)
    assert ctrl.update(0.0, 0.5, 0.0, 0.0, 0.0, 0.0)["delta_main"] == pytest.approx(0.5, abs=0.01)


def test_actuator_delay_is_specified_in_seconds():
    for dt, steps in ((0.01, 2), (0.005, 4), (0.02, 1)):
        c = HydrofoilController(GAINS_1, dt=dt, actuator_params={"rate_limit_deg_s": 720.0, "tau": 0.02, "delay_s": 0.02})
        assert c.main_actuator.delay_steps == steps


# --- anti-windup ------------------------------------------------------------

def test_anti_windup_keeps_integrating_when_error_desaturates():
    pid = PID(Kp=0.1, Ki=1.0, Kd=0.0, dt=0.01, output_min=-1.0, output_max=1.0)
    pid._integral = 5.0                       # output saturated high ...
    pid.update(error=-0.01, measurement=0.0)  # ... and the error is pulling it back down
    assert pid._integral < 5.0                # the old rule undid this step and stayed at 5.0


def test_anti_windup_freezes_integrator_when_error_pushes_deeper():
    pid = PID(Kp=0.1, Ki=1.0, Kd=0.0, dt=0.01, output_min=-1.0, output_max=1.0)
    pid._integral = 5.0
    pid.update(error=+0.01, measurement=0.0)
    assert pid._integral == 5.0


# --- surge derivative sign --------------------------------------------------

def test_surge_derivative_opposes_acceleration():
    s = SurgeController(Kp=0.0, Ki=0.0, Kd=1.0, dt=1.0, max_thrust=100.0)
    s.update(4.0, 0.0)
    assert s.update(4.0, 1.0) < 0.0           # speed rising -> D term brakes (was positive feedback)


# --- rigid body: Munk moment ------------------------------------------------

def test_added_mass_coriolis_munk_moment_and_passivity():
    rb = RigidBody(25.0, np.diag([2.0, 5.0, 5.0]), added_mass_heave=24.0, added_inertia_pitch=3.5)
    nu = np.array([4.0, 0.2, 0.3, 0.1, -0.2, 0.15])
    assert nu @ rb.C(nu) @ nu == pytest.approx(0.0, abs=1e-9)          # no energy created
    nu2 = np.array([4.0, 0, 0.5, 0, 0, 0])
    assert (rb.C(nu2) @ nu2)[4] == pytest.approx(-24.0 * 4.0 * 0.5)   # -m_a u w (destabilising Munk moment)


# --- foil -------------------------------------------------------------------

def test_foil_ignores_spanwise_flow():
    f = Foil(FoilGeometry(0.15, 0.6), POLAR, r_attach=np.array([0, 0.3, 0.5]))
    a = f.generalized_force(np.array([4.0, 0.0, 0.2, 0, 0, 0]), delta=5.0)
    b = f.generalized_force(np.array([4.0, 3.0, 0.2, 0, 0, 0]), delta=5.0)
    assert np.allclose(a, b)


def test_finite_span_factor_matches_induced_drag_assumptions():
    f = Foil(FoilGeometry(0.15, 0.6), POLAR, r_attach=np.zeros(3), oswald_efficiency=0.8)
    a2d = POLAR.cl_slope_per_deg * 180.0 / np.pi
    assert f.lift_slope_factor == pytest.approx(1.0 / (1.0 + a2d / (np.pi * 0.8 * 4.0)))


def test_vertical_strut_side_force_is_restoring():
    strut = Foil(FoilGeometry(0.12, 0.5), POLAR, r_attach=np.array([0, 0, 0.25]), normal_axis=np.array([0, 1.0, 0]))
    tau = strut.generalized_force(np.array([4.0, 0.5, 0, 0, 0, 0]))   # moving to starboard
    assert tau[1] < 0.0                                                 # pushed back to port


# --- hull -------------------------------------------------------------------

def test_hull_draft_follows_attitude():
    """Rolling swings the hull-bottom point (0.6 m above the CoG) sideways, so it sits LOWER in NED z."""
    level = hull_draft(np.array([0, 0, 0.7, 0.0, 0, 0]), 0.6, 0.3, 0.0)
    rolled = hull_draft(np.array([0, 0, 0.7, 0.6, 0, 0]), 0.6, 0.3, 0.0)
    assert level == pytest.approx(0.1)
    assert rolled == pytest.approx(0.7 - 0.6 * np.cos(0.6))


def test_hull_buoyancy_gives_restoring_roll_moment():
    eta = np.array([0, 0, 0.7, np.radians(10), 0, 0])        # rolled to starboard
    assert hull_buoyancy_force(eta, 0.6, 0.6, 0.3)[3] < 0.0   # moment rolls it back to port


# --- waves ------------------------------------------------------------------

def test_wave_period_follows_deep_water_dispersion():
    w = AiryWave(wavelength=5.0)
    assert w.omega ** 2 == pytest.approx(G * w.k)


def test_beam_seas_load_left_and_right_foils_differently_head_seas_do_not():
    vp = VehiclePhysics(dict(DEFAULT_PARAMS))
    eta = np.zeros(6)
    head = vp.wave_sample(eta, 0.3, AiryWave(0.05, 5.0, direction=0.0))
    beam = vp.wave_sample(eta, 0.3, AiryWave(0.05, 5.0, direction=np.pi / 2))
    assert head.foil_surface_z["left"] == pytest.approx(head.foil_surface_z["right"])
    assert beam.foil_surface_z["left"] != pytest.approx(beam.foil_surface_z["right"])


# --- closed loop (the tests that would have caught the roll-loop bug) ---------

def _settle(T=60.0, params=None):
    p = dict(DEFAULT_PARAMS if params is None else params)
    vp = VehiclePhysics(p)
    s, t = vp.rest_state(), 0.0
    for _ in range(int(T / p["dt"])):
        s, _, _, _, _ = vp.step(s, t, CalmWater()); t += p["dt"]
    return p, vp, s, t


def test_takeoff_reaches_foilborne_and_speed():
    p, vp, s, t = _settle(30.0)
    assert vp.step(s, t, CalmWater())[3] <= 1e-6        # draft zero
    assert s[6] == pytest.approx(p["target_speed"], abs=0.1)


def test_roll_disturbance_is_rejected():
    p, vp, s, t = _settle(60.0)
    s[3] = np.radians(5.0)
    for _ in range(int(10.0 / p["dt"])):
        s, _, _, _, _ = vp.step(s, t, CalmWater()); t += p["dt"]
    assert abs(np.degrees(s[3])) < 0.5


def test_bank_command_is_tracked_without_losing_height():
    p, vp, s, t = _settle(60.0)
    p["phi_ref"] = 8.0
    worst_draft = 0.0
    for _ in range(int(15.0 / p["dt"])):
        s, _, _, d, _ = vp.step(s, t, CalmWater()); t += p["dt"]
        worst_draft = max(worst_draft, d)
    assert abs(np.degrees(s[3]) - 8.0) < 1.5
    assert worst_draft == 0.0
    assert np.degrees(s[5]) > 5.0                         # it actually turned (positive heading change)


def test_oblique_waves_do_not_destabilise_roll():
    p = dict(DEFAULT_PARAMS)
    vp = VehiclePhysics(p)
    s, t = vp.rest_state(), 0.0
    wave = AiryWave(0.05, 5.0, direction=np.radians(45.0))
    worst = 0.0
    for _ in range(int(60.0 / p["dt"])):
        s, _, _, _, _ = vp.step(s, t, wave); t += p["dt"]
        worst = max(worst, abs(np.degrees(s[3])))
    assert worst < 10.0


def test_closed_form_rotation_matches_matmul_form():
    from hydrofoil.dynamics.kinematics import R_body_to_ned, Rx, Ry, Rz
    for phi, theta, psi in ((0.1, -0.3, 2.0), (-1.2, 0.7, -3.0), (0.0, 0.0, 0.0)):
        assert np.allclose(R_body_to_ned(phi, theta, psi), Rz(psi) @ Ry(theta) @ Rx(phi), atol=1e-12)


# --- distributed hull hydrostatics vs classical theory (Fossen ch.4) -----------

def _stiffness(H, eta0, idx, k, e=1e-4):
    v = np.zeros(6); v[idx] = e
    return -(H.buoyancy(eta0 + v, 0.0)[k] - H.buoyancy(eta0 - v, 0.0)[k]) / (2 * e)


def test_hull_hydrostatic_stiffness_matches_metacentric_theory():
    from hydrofoil.constants import RHO_WATER, G
    from hydrofoil.forces.hull import HullHydrostatics
    L, B, h, dmax, m = 2.0, 0.30, 0.6, 0.3, 25.0
    H = HullHydrostatics(L, B, 0.0, h, dmax)
    A = L * B; d = m / (RHO_WATER * A); V = A * d
    eta0 = np.array([0, 0, h + d, 0, 0, 0.0])
    KB = h + d / 2                                     # centre of buoyancy above the CoG
    assert _stiffness(H, eta0, 2, 2) == pytest.approx(RHO_WATER * G * A, rel=1e-6)                       # heave
    assert _stiffness(H, eta0, 3, 3) == pytest.approx(RHO_WATER * G * V * (KB + L * B**3 / 12 / V), rel=0.02)   # roll, GM_T
    assert _stiffness(H, eta0, 4, 4) == pytest.approx(RHO_WATER * G * V * (KB + B * L**3 / 12 / V), rel=0.02)  # pitch, GM_L
    assert H.buoyancy(eta0, 0.0)[2] == pytest.approx(-m * G)                                             # floats at the right draft


def test_hull_buoyancy_force_is_linear_in_draft_when_level():
    from hydrofoil.forces.hull import HullHydrostatics
    H = HullHydrostatics(2.0, 0.30, 0.0, 0.6, 0.3)
    f1 = -H.buoyancy(np.array([0, 0, 0.65, 0, 0, 0.0]), 0.0)[2]
    f2 = -H.buoyancy(np.array([0, 0, 0.70, 0, 0, 0.0]), 0.0)[2]
    assert f2 == pytest.approx(2 * f1)


def test_hull_rolled_partially_emerged_buoyancy_is_restoring_and_vertical_force_positive():
    from hydrofoil.forces.hull import HullHydrostatics
    H = HullHydrostatics(2.0, 0.30, 0.0, 0.6, 0.3)
    tau = H.buoyancy(np.array([0, 0, 0.62, np.radians(15), 0, 0.0]), 0.0)
    assert tau[3] < 0.0          # starboard-down roll -> moment back to port
    assert tau[2] < 0.0          # still lifts (up = -z)


def test_floating_hull_rocking_is_damped():
    p = dict(DEFAULT_PARAMS); p["target_speed"] = 0.0
    vp = VehiclePhysics(p)
    s, t = vp.rest_state(), 0.0
    th = []
    for _ in range(1500):
        s, _, _, _, _ = vp.step(s, t, CalmWater()); t += p["dt"]
        th.append(np.degrees(s[4]))
    assert max(abs(x) for x in th[-300:]) < 0.5          # initial 2 deg rocking has died away
