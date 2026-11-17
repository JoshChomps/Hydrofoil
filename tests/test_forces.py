"""
Unit tests for physics models.

Run with: python -m pytest tests/ -v
"""

import numpy as np
import pytest
from hydrofoil.dynamics.kinematics import J, R_body_to_ned
from hydrofoil.dynamics.rigid_body import RigidBody, skew
from hydrofoil.forces.foil import Foil, FoilGeometry, FoilTables
from hydrofoil.forces.buoyancy import buoyancy_force, gravity_force

POLAR = FoilTables.naca0009()


# --- NACA 0009 polar ---

def test_naca0009_loads():
    assert len(POLAR.aoa_deg) > 100  # 119 rows in the XFLR5 file

def test_naca0009_symmetric():
    """Symmetric foil: CL(+α) = -CL(-α), CD(+α) = CD(-α)."""
    for a in [1.0, 3.0, 5.0, 8.0, 12.0, 16.0]:
        assert abs(POLAR.cl(a) + POLAR.cl(-a)) < 1e-4, f"CL asymmetry at {a} deg"
        assert abs(POLAR.cd(a) - POLAR.cd(-a)) < 1e-4, f"CD asymmetry at {a} deg"

def test_naca0009_zero_lift_at_zero_aoa():
    assert abs(POLAR.cl(0.0)) < 1e-6

def test_naca0009_cl_slope():
    # Thin-airfoil theory: 2π/rad = 0.1097/deg. NACA 0009 should be close.
    slope = POLAR.cl_slope_per_deg
    assert 0.10 < slope < 0.12, f"CL slope {slope:.4f}/deg outside expected range"

def test_naca0009_stall():
    # CL should peak near 14-15 deg and not extrapolate past table bounds
    cl_at_20 = POLAR.cl(20.0)   # beyond table — should clamp to last entry
    assert cl_at_20 < 1.5, "Clamping past stall should not produce unrealistically high CL"


# --- Kinematics ---

def test_rotation_matrix_orthogonal():
    R = R_body_to_ned(0.1, -0.05, 0.3)
    assert np.allclose(R @ R.T, np.eye(3), atol=1e-10)
    assert np.allclose(np.linalg.det(R), 1.0, atol=1e-10)


def test_J_at_zero_angles_is_identity():
    eta = np.zeros(6)
    Jm = J(eta)
    assert np.allclose(Jm, np.eye(6), atol=1e-10)


# --- Rigid body ---

def test_skew_antisymmetric():
    v = np.array([1.0, 2.0, 3.0])
    S = skew(v)
    assert np.allclose(S + S.T, 0)


def test_rigid_body_M_symmetric():
    rb = RigidBody(mass=25.0, I_b=np.diag([2.0, 5.0, 5.0]))
    assert np.allclose(rb.M, rb.M.T)


def test_rigid_body_M_positive_definite():
    rb = RigidBody(mass=25.0, I_b=np.diag([2.0, 5.0, 5.0]))
    eigvals = np.linalg.eigvalsh(rb.M)
    assert np.all(eigvals > 0)


def test_rigid_body_zero_force_zero_accel():
    rb = RigidBody(mass=25.0, I_b=np.diag([2.0, 5.0, 5.0]))
    nu = np.zeros(6)
    nu_dot = rb.nu_dot(nu, np.zeros(6))
    assert np.allclose(nu_dot, 0)


# --- Foil forces ---

def test_foil_zero_speed_returns_zero():
    foil = Foil(FoilGeometry(0.15, 0.6), POLAR, r_attach=np.array([0, 0, 0.5]))
    nu = np.zeros(6)
    tau = foil.generalized_force(nu)
    assert np.allclose(tau, 0)


def test_foil_produces_upward_force_at_positive_aoa():
    foil = Foil(FoilGeometry(0.15, 0.6), POLAR, r_attach=np.array([0, 0, 0.5]))
    # Positive flap delta → positive AoA → upward lift (negative NED z force)
    nu = np.array([4.0, 0, 0, 0, 0, 0])
    tau = foil.generalized_force(nu, delta=5.0)
    assert tau[2] < 0, f"Expected upward (negative z) lift, got Z={tau[2]:.3f}"

def test_foil_sinking_vehicle_produces_restoring_lift():
    foil = Foil(FoilGeometry(0.15, 0.6), POLAR, r_attach=np.array([0, 0, 0.5]))
    # Vehicle sinking (w > 0 in NED) → positive AoA → upward lift → restoring
    nu = np.array([4.0, 0, 0.5, 0, 0, 0])   # 0.5 m/s heave downward
    tau = foil.generalized_force(nu, delta=0.0)
    assert tau[2] < 0, f"Sinking should produce upward lift, got Z={tau[2]:.3f}"


def test_foil_drag_opposes_motion():
    foil = Foil(FoilGeometry(0.15, 0.6), POLAR, r_attach=np.array([0, 0, 0.5]))
    nu = np.array([4.0, 0, 0, 0, 0, 0])
    tau = foil.generalized_force(nu)
    # Drag should oppose forward motion = negative X force
    assert tau[0] < 0, f"Expected negative X drag, got X={tau[0]:.3f}"


def test_foil_reverse_flow_continuous_across_zero_vertical_velocity():
    foil = Foil(FoilGeometry(0.28, 1.1, aoa_offset=3.0), POLAR, r_attach=np.array([0, 0.6, 1.75]), flap_effectiveness=0.342)
    z_pos = foil.generalized_force(np.array([-0.5, 0.0, +0.001, 0, 0, 0]))[2]
    z_neg = foil.generalized_force(np.array([-0.5, 0.0, -0.001, 0, 0, 0]))[2]
    assert abs(z_pos - z_neg) < 1.0


def test_foil_flap_stall_and_ventilation_reduce_lift_smoothly():
    f_lin = Foil(FoilGeometry(0.28, 1.1, aoa_offset=3.0), POLAR, r_attach=np.array([0, 0.6, 1.75]), flap_effectiveness=0.342)
    f_sep = Foil(FoilGeometry(0.28, 1.1, aoa_offset=3.0), POLAR, r_attach=np.array([0, 0.6, 1.75]),
                 flap_effectiveness=0.342, flap_stall_deg=15.0)
    nu = np.array([5.5, 0.0, 0.0, 0.0, 0.0, 0.0])
    # Identical below 15 deg
    assert np.allclose(f_lin.generalized_force(nu, delta=12.0), f_sep.generalized_force(nu, delta=12.0))
    # Above 15 deg: lift is attenuated and separation drag rises
    t_lin20 = f_lin.generalized_force(nu, delta=20.0)
    t_sep20 = f_sep.generalized_force(nu, delta=20.0)
    assert abs(t_sep20[2]) < abs(t_lin20[2])
    assert -t_sep20[0] > -t_lin20[0]
    # Ventilation collapses lift and adds cavity drag
    t_clean = f_sep.generalized_force(nu, delta=5.0, ventilation=0.0)
    t_vent = f_sep.generalized_force(nu, delta=5.0, ventilation=1.0)
    assert abs(t_vent[2]) < 0.6 * abs(t_clean[2]) and -t_vent[0] > -t_clean[0]


def test_hull_froude_drag_hump_and_roll_added_inertia():
    from hydrofoil.forces.hull import HullHydrostatics
    from hydrofoil.dynamics.rigid_body import RigidBody, strip_added_mass_roll
    hull = HullHydrostatics(3.6, 1.2, 0.0, -0.35, 0.3)
    d_base = abs(hull.drag(np.array([2.85, 0, 0, 0, 0, 0.0]), 0.1, 120.0, 0.0)[0])
    d_hump = abs(hull.drag(np.array([2.85, 0, 0, 0, 0, 0.0]), 0.1, 120.0, 0.0, hull_length=3.6, hump_amp=0.8)[0])
    assert d_hump > 1.7 * d_base
    assert hull.drag(np.array([8.0, 0, 0, 0, 0, 0.0]), 0.0, 120.0, 0.0, hull_length=3.6, hump_amp=0.8)[0] == 0.0
    ir = strip_added_mass_roll([(0.28, 1.1, -0.6), (0.28, 1.1, 0.6)])
    assert 60.0 < ir < 68.0
    rb = RigidBody(275.0, np.diag([80.0, 260.0, 280.0]), added_mass_heave=140.0, added_inertia_pitch=30.0, added_inertia_roll=ir)
    nu = np.array([7.0, 0.2, 0.3, 0.4, -0.2, 0.1])
    assert abs(float(nu @ rb.C(nu) @ nu)) < 1e-10


# --- Buoyancy / gravity ---

def test_buoyancy_upward():
    tau = buoyancy_force(displaced_volume=0.025, eta=np.zeros(6))
    assert tau[2] < 0   # upward = negative NED z


def test_gravity_downward_at_zero_angle():
    eta = np.zeros(6)
    tau = gravity_force(mass=25.0, eta=eta)
    assert tau[2] > 0   # downward = positive NED z


def test_buoyancy_gravity_balance_at_neutral():
    m = 25.0
    rho = 1025.0
    g = 9.81
    V = m / rho
    eta = np.zeros(6)
    tau_b = buoyancy_force(V, eta, rho, g)
    tau_g = gravity_force(m, eta, g)
    net_z = tau_b[2] + tau_g[2]
    assert abs(net_z) < 1e-6, f"Neutral buoyancy should be zero net, got {net_z:.6f}"
