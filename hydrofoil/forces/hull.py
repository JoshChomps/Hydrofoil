"""
Simplified wall-sided hull buoyancy and drag, for modeling the transition
between displacement (floating) and foilborne (flying) regimes.

The original forces/buoyancy.py::buoyancy_force() assumes constant displaced
volume — correct for a vehicle that is always fully submerged, but it can't
represent a hull rising out of the water as foil lift takes over. This module
adds that: buoyancy that depends on how deep the hull currently is (its
"draft"), tapering to zero once the hull clears the surface.

Geometry convention (NED, z positive down):
    hull_bottom_z = eta[2] (CoG depth) - hull_offset_above_cog
so a positive hull_offset_above_cog means the hull rides above the CoG
reference point (as it does on a strut/mast-mounted foil vehicle). draft is
how far the hull bottom sits below the calm-water surface (z=0), clamped to
[0, max_draft] — a linear (wall-sided) approximation valid near the design
waterline, not a full hull-form model.
"""

import numpy as np

from hydrofoil.constants import RHO_WATER, G
from hydrofoil.dynamics.kinematics import R_body_to_ned


def hull_draft(eta: np.ndarray, hull_offset_above_cog: float, max_draft: float,
                water_surface_z: float = 0.0) -> float:
    """
    Depth of the hull bottom below the (possibly wave-displaced) local water
    surface [m], clamped to [0, max_draft]. water_surface_z is the surface's
    current NED z-position (0 = calm water datum; negative = surface risen).
    """
    # Hull-bottom reference point sits hull_offset above the CoG along body -z, so its
    # NED depth follows the attitude (previously attitude was ignored here while
    # the foil submergence calculation did rotate -- inconsistent).
    R = R_body_to_ned(eta[3], eta[4], eta[5])
    hull_bottom_z = eta[2] + (R @ np.array([0.0, 0.0, -hull_offset_above_cog]))[2]
    return float(np.clip(hull_bottom_z - water_surface_z, 0.0, max_draft))


def hull_buoyancy_force(
    eta: np.ndarray,
    hull_offset_above_cog: float,
    waterplane_area: float,
    max_draft: float,
    rho_water: float = RHO_WATER,
    g: float = G,
    water_surface_z: float = 0.0,
) -> np.ndarray:
    """
    Buoyancy force that grows linearly with draft and vanishes once the hull
    is clear of the water — replaces constant-volume buoyancy near the surface.
    """
    draft = hull_draft(eta, hull_offset_above_cog, max_draft, water_surface_z)
    volume = waterplane_area * draft

    phi, theta, psi = eta[3], eta[4], eta[5]
    R = R_body_to_ned(phi, theta, psi)

    F_b = rho_water * g * volume
    f_ned = np.array([0.0, 0.0, -F_b])   # upward = -z in NED
    f_body = R.T @ f_ned

    # Buoyancy acts at the centre of buoyancy of the submerged volume, which sits
    # above the CoG (hull_offset to the hull bottom, plus half the draft). The
    # force is not through the CoG, so tilting the hull produces a restoring
    # moment r_cb x F (previously applied at the CoG -> no hydrostatic stiffness).
    r_cb = np.array([0.0, 0.0, -(hull_offset_above_cog + 0.5 * draft)])
    tau = np.zeros(6)
    tau[:3] = f_body
    tau[3:] = np.array([r_cb[1] * f_body[2] - r_cb[2] * f_body[1],
                        r_cb[2] * f_body[0] - r_cb[0] * f_body[2],
                        r_cb[0] * f_body[1] - r_cb[1] * f_body[0]])
    return tau


def hull_drag_force(
    nu: np.ndarray,
    eta: np.ndarray,
    hull_offset_above_cog: float,
    max_draft: float,
    drag_coeff: float = 15.0,
    water_surface_z: float = 0.0,
) -> np.ndarray:
    """
    Placeholder quadratic hull drag (wave-making + skin friction), scaled by
    how wetted the hull currently is (1.0 at full design draft, 0 once
    foilborne). Real hulls show a drag "hump" peaking just before takeoff;
    this linear-with-draft model captures the qualitative transition, not
    the hump's exact shape — replace with tow-tank/CFD data if precision matters.
    """
    draft = hull_draft(eta, hull_offset_above_cog, max_draft, water_surface_z)
    wetted_frac = draft / max_draft if max_draft > 0 else 0.0

    u = nu[0]
    Fx = -drag_coeff * wetted_frac * u * abs(u)

    tau = np.zeros(6)
    tau[0] = Fx
    return tau
