import numpy as np

from hydrofoil.actuation.hinge import thin_airfoil_coeffs, HingeMoment, SurfaceGeometry
from hydrofoil.actuation.drive import DriveSpec, electrical_demand, limits_at_output


def test_thin_airfoil_solver_matches_closed_forms():
    c0 = thin_airfoil_coeffs(0.0)
    assert abs(c0["cl_alpha"] - 2 * np.pi) < 1e-3
    assert abs(c0["x_ac"] - 0.25) < 1e-3
    for E in (0.2, 0.3, 0.4):
        th = np.arccos(1 - 2 * (1 - E))
        tau = 1 - (th - np.sin(th)) / np.pi       # Glauert flap effectiveness
        c = thin_airfoil_coeffs(E)
        assert abs(c["tau"] - tau) < 2e-3
        assert c["ch_alpha"] < 0 and c["ch_delta"] < 0   # load pushes the TE back up


def test_flap_hinge_moment_scales_with_q_and_opposes_deflection():
    hm = HingeMoment(SurfaceGeometry(chord=0.15, span=0.6, layout="flap", flap_frac=0.4))
    m4 = hm.steady(4.0, 0.0, np.radians(10))
    m8 = hm.steady(8.0, 0.0, np.radians(10))
    assert m4 < 0
    assert abs(m8 / m4 - 4.0) < 1e-9
    assert hm.damping(8.0) == 2 * hm.damping(4.0) > 0


def test_all_moving_pivot_ahead_of_ac_is_restoring():
    ahead = HingeMoment(SurfaceGeometry(chord=0.1, span=0.3, layout="all_moving", pivot_frac=0.20))
    behind = HingeMoment(SurfaceGeometry(chord=0.1, span=0.3, layout="all_moving", pivot_frac=0.35))
    a = np.radians(4)
    assert ahead.steady(5.0, a, a) < 0 < behind.steady(5.0, a, a)


def test_drive_holding_current_and_regen_direction():
    spec = DriveSpec()
    e = electrical_demand(spec, 0.0, 0.0, 2.0)            # hold 2 N m, no motion
    i_expected = 2.0 / (spec.gear.ratio * spec.gear.efficiency) / spec.motor.kt
    assert abs(float(e["i_q"]) - i_expected) < 1e-9
    back = electrical_demand(spec, -1.0, 0.0, 2.0)        # load driving the motor backwards
    assert float(back["p_bus"]) < float(electrical_demand(spec, 1.0, 0.0, 2.0)["p_bus"])


def test_reflected_inertia_and_rate_limit_scale_with_ratio():
    a, b = DriveSpec(), DriveSpec()
    b.gear.ratio = 40.0
    la, lb = limits_at_output(a, 2e-3), limits_at_output(b, 2e-3)
    assert abs(lb["j_reflected"] / la["j_reflected"] - 4.0) < 1e-9
    assert abs(la["rate_max"] / lb["rate_max"] - 2.0) < 1e-9


def test_flap_effectiveness_follows_geometry_and_keeps_the_reference():
    from hydrofoil.forces.foil import main_flap_effectiveness, MAIN_FOIL_FLAP_EFFECTIVENESS
    from hydrofoil.params import DEFAULT_PARAMS, MANNED_PARAMS
    from hydrofoil.vehicle import VehiclePhysics
    assert abs(main_flap_effectiveness(0.4, 0.5) - MAIN_FOIL_FLAP_EFFECTIVENESS) < 1e-12
    effs = [main_flap_effectiveness(cf, 0.5) for cf in (0.2, 0.3, 0.4, 0.5)]
    assert all(b > a for a, b in zip(effs, effs[1:]))                 # bigger flap, more lift per degree
    assert abs(main_flap_effectiveness(0.3, 1.0) - 2 * main_flap_effectiveness(0.3, 0.5)) < 1e-12
    for p in (DEFAULT_PARAMS, MANNED_PARAMS):                          # defaults unchanged
        vp = VehiclePhysics(dict(p, controller="pid", sensors="ideal"))
        assert abs(vp.left_foil.flap_effectiveness - MAIN_FOIL_FLAP_EFFECTIVENESS) < 1e-12


def test_hinge_setback_balances_the_flap():
    from hydrofoil.actuation.hinge import thin_airfoil_coeffs
    plain = thin_airfoil_coeffs(0.3)
    assert thin_airfoil_coeffs(0.3, setback=0.0)["ch_delta"] == plain["ch_delta"]
    ch = [thin_airfoil_coeffs(0.3, setback=s)["ch_delta"] for s in (0.0, 0.1, 0.2, 0.3)]
    assert all(b > a for a, b in zip(ch, ch[1:])) and ch[-1] < 0      # less moment, still not overbalanced
    assert thin_airfoil_coeffs(0.3, setback=0.4)["ch_delta"] > 0      # overbalanced: moment reverses
    assert thin_airfoil_coeffs(0.3, setback=0.2)["tau"] == plain["tau"]   # lift is unaffected


def test_flap_design_tool_mechanics():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
    import flap_design as fd
    m = fd.mechanics(28.0)
    assert 12.0 < m["solid_shaft_mm"] < 13.5                           # 16 T / (pi d^3) = 70 MPa
    w = fd.gear_window(20.0, 150.0, 5.0)
    assert w["N_min_torque"] < 25 < w["N_max_speed"]
