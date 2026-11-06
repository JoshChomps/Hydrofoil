"""
Hydrodynamic hinge moment on a control surface: the torque the actuator has to hold and drive.

Two layouts are supported:

* ``flap``       plain trailing-edge flap of chord fraction E hinged at x_h = (1 - E) c
                 (this is what the main foil halves are drawn as in the viewer).
* ``all_moving`` the whole foil rotates about a pivot at x_p = pivot_frac * c
                 (the elevator, and the alternative for the main wing).

Steady part
-----------
Coefficients come from 2-D thin-airfoil theory, solved numerically with a lumped-vortex
panel method (vortex at the panel 1/4 point, collocation at 3/4) so nothing is copied
from a table: it self-checks against C_l = 2*pi*alpha, the aerodynamic centre at c/4
and Glauert's flap effectiveness tau = 1 - (theta_h - sin theta_h)/pi
(``tests/test_actuation.py``).

    flap:        C_h = C_h_alpha * alpha + C_h_delta * delta,   M_h = q * S_f * c_f * C_h
    all-moving:  M_p = q * S * c * C_L * (x_p - x_ac)/c         (positive = nose up / TE down)

Sign convention: positive moment acts in the +delta direction (trailing edge DOWN, the same
direction as a positive flap / more lift). Hydrodynamic load on a flap is therefore negative
(it pushes the trailing edge up, toward the stream); the actuator supplies the opposite.

Thin-airfoil theory over-predicts hinge moments of real sections because the boundary layer
thickens toward the trailing edge; ``viscous_factor`` (PLACEHOLDER 0.8) scales the steady
coefficients down. The 3-D lift loss (Helmbold) and the free-surface factor already used by
``forces/foil.py`` are applied on top so this stays consistent with the lift the sim computes.

Unsteady part (quasi-steady Theodorsen, C(k) = 1)
------------------------------------------------
Water is ~800x denser than air, so the fluid the surface drags along when it rotates is not
negligible next to the actuator inertia:

    all-moving about a = (pivot_frac*c - c/2)/(c/2) half-chords from mid-chord, b = c/2, per span:
        I_added = pi rho b^4 (1/8 + a^2)
        B_aero  = pi rho b^3 V (1/2 - a)  -  2 pi rho V b^3 (a + 1/2)(1/2 - a)
    flap: approximated as an isolated plate hinged at its own leading edge (a = -1, b = c_f/2):
        I_added = (9/8) pi rho b^4,   B_aero = 3 pi rho b^3 V

The flap formula ignores the main element ahead of it (Theodorsen's T-functions would add
that), so treat its unsteady terms as order-of-magnitude.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import numpy as np

from hydrofoil.constants import RHO_WATER


@lru_cache(maxsize=32)
def thin_airfoil_coeffs(flap_frac: float, n: int = 400) -> dict:
    """
    2-D thin-airfoil coefficients for a unit-chord plate with a plain flap of chord fraction
    ``flap_frac`` (0 = no flap). Per radian. Hinge moment is referenced to q * c_f^2 (per span).

    Returns dict: cl_alpha, cl_delta, tau (= cl_delta/cl_alpha), x_ac,
                  ch_alpha, ch_delta   (flap only; 0 when flap_frac == 0).
    """
    h = 1.0 / n
    xv = (np.arange(n) + 0.25) * h          # vortex positions
    xc = (np.arange(n) + 0.75) * h          # collocation points
    A = 1.0 / (2.0 * np.pi * (xc[:, None] - xv[None, :]))
    x_h = 1.0 - flap_frac
    on_flap_c = xc > x_h
    on_flap_v = xv > x_h

    # Non-penetration with unit free stream: sum_j A_ij G_j = alpha (+ delta on the flap).
    g_alpha = np.linalg.solve(A, np.ones(n))
    g_delta = np.linalg.solve(A, on_flap_c.astype(float))

    # Per unit span, rho = V = 1: lift = sum(G), so C_l = 2 sum(G) (q = 1/2).
    cl_a, cl_d = 2.0 * g_alpha.sum(), 2.0 * g_delta.sum()
    x_ac = float((g_alpha * xv).sum() / g_alpha.sum())

    out = {"cl_alpha": cl_a, "cl_delta": cl_d, "tau": cl_d / cl_a, "x_ac": x_ac,
           "ch_alpha": 0.0, "ch_delta": 0.0}
    if flap_frac > 0:
        cf = flap_frac
        arm = np.where(on_flap_v, xv - x_h, 0.0)
        # Upward load aft of the hinge pushes the TE up = negative in the TE-down convention.
        out["ch_alpha"] = -2.0 * (g_alpha * arm).sum() / cf**2
        out["ch_delta"] = -2.0 * (g_delta * arm).sum() / cf**2
    return out


@dataclass
class SurfaceGeometry:
    chord: float               # full foil chord [m]
    span: float                # span carried by THIS actuator [m]
    layout: str = "flap"       # "flap" or "all_moving"
    flap_frac: float = 0.4     # flap chord / chord (flap layout)
    pivot_frac: float = 0.25   # pivot position / chord from the LE (all-moving layout)
    viscous_factor: float = 0.8   # PLACEHOLDER real/thin-airfoil ratio for steady hinge moments
    j_struct: float = 0.0      # structural inertia of the moving part about its hinge [kg m^2]


class HingeMoment:
    """Hydrodynamic moment about one actuator's hinge, as a function of flow and motion."""

    def __init__(self, geom: SurfaceGeometry, rho: float = RHO_WATER):
        self.g = geom
        self.rho = rho
        c, s = geom.chord, geom.span
        if geom.layout == "flap":
            co = thin_airfoil_coeffs(geom.flap_frac)
            cf = geom.flap_frac * c
            b = cf / 2.0
            self._ref = s * cf * cf                       # S_f * c_f
            self.ch_alpha = geom.viscous_factor * co["ch_alpha"]
            self.ch_delta = geom.viscous_factor * co["ch_delta"]
            self.i_added = 9.0 / 8.0 * np.pi * rho * b**4 * s
            self._b_per_V = 3.0 * np.pi * rho * b**3 * s
        elif geom.layout == "all_moving":
            co = thin_airfoil_coeffs(0.0)
            b = c / 2.0
            a = (geom.pivot_frac * c - b) / b
            self._ref = s * c * c                         # S * c
            # moment about the pivot per unit C_L: lift acts at x_ac
            self._arm = geom.viscous_factor * (geom.pivot_frac - co["x_ac"])
            self.i_added = np.pi * rho * b**4 * (0.125 + a**2) * s
            self._b_per_V = (np.pi * rho * b**3 * (0.5 - a)
                             - 2.0 * np.pi * rho * b**3 * (a + 0.5) * (0.5 - a)) * s
        else:
            raise ValueError(f"unknown layout {geom.layout!r}")
        self.coeffs = co

    def steady(self, V: float, alpha: float, delta: float, cl: float | None = None,
               lift_factor: float = 1.0) -> float:
        """
        Steady hinge moment [N m], + = trailing edge down.

        V      flow speed in the foil plane [m/s]
        alpha  geometric angle of attack of the fixed part (flap layout) or of the whole
               surface including its deflection (all-moving) [rad]
        delta  flap deflection [rad] (flap layout only)
        cl     3-D lift coefficient the sim already computed (all-moving layout); if None,
               2*pi*alpha*lift_factor is used.
        lift_factor  Helmbold x free-surface factor, scales the circulatory load (flap layout)
        """
        q = 0.5 * self.rho * V * V
        if self.g.layout == "flap":
            return q * self._ref * lift_factor * (self.ch_alpha * alpha + self.ch_delta * delta)
        if cl is None:
            cl = 2.0 * np.pi * alpha * lift_factor
        # For an all-moving surface, +delta (TE down) IS nose-up. Lift acting at the AC ahead
        # of the pivot (x_ac < x_p) is nose-up => positive => divergent (the actuator must hold it
        # back); a pivot ahead of the AC weathervanes the surface back to zero lift.
        return q * self._ref * cl * self._arm

    def damping(self, V: float) -> float:
        """Hydrodynamic rotational damping coefficient [N m s/rad] (opposes delta_dot)."""
        return self._b_per_V * abs(V)

    def total(self, V, alpha, delta, delta_dot, delta_ddot, cl=None, lift_factor=1.0) -> float:
        """Total hydrodynamic moment on the surface [N m], + = TE down (includes added-mass reaction)."""
        return (self.steady(V, alpha, delta, cl, lift_factor)
                - self.damping(V) * delta_dot
                - self.i_added * delta_ddot)
