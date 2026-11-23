"""
6-DOF rigid-body equations of motion for a marine vehicle.

Follows Fossen (2011) Chapters 3–4:
    M_RB * nu_dot + C_RB(nu) * nu = tau

where:
    M_RB = rigid-body mass/inertia matrix (6x6, constant)
    C_RB = Coriolis/centripetal matrix (6x6, function of nu)
    tau  = generalized external forces and moments [N, N, N, Nm, Nm, Nm]

Diagonal hydrodynamic added mass (heave + pitch only) is folded directly
into M here via optional RigidBody constructor arguments — see
strip_added_mass_heave_pitch() below for how a caller derives them from its
lifting surfaces' geometry. This is a deliberate exception to the usual
"added by the caller" split: Fossen's nu_dot solve is against a single
combined M = M_RB + M_A, so keeping the diagonal M_A terms in the same
matrix M_RB builds gets callers a correct combined solve for free.
Off-diagonal terms and surge added mass are NOT modeled — diagonal M_A covers
M[1,1] (sway), M[2,2] (heave), M[3,3] (roll), M[4,4] (pitch), and M[5,5] (yaw).
The matching added-mass Coriolis term C_A (pitch and yaw Munk moments) IS included
in C(), preserving exact passivity nu^T C(nu) nu == 0. Damping (D) is still added
by the caller.
"""

import numpy as np

from hydrofoil.constants import RHO_WATER


def skew(v: np.ndarray) -> np.ndarray:
    """3x3 skew-symmetric matrix for cross product: skew(a) @ b == a x b."""
    return np.array([[ 0,    -v[2],  v[1]],
                     [ v[2],  0,    -v[0]],
                     [-v[1],  v[0],  0   ]])


def strip_added_mass_roll(strips, rho: float = RHO_WATER) -> float:
    """
    2D-strip-theory hydrodynamic added inertia in roll about body x, for one
    or more horizontal lifting surfaces.

    Parameters
    ----------
    strips : iterable of (chord, span, y) — one entry per lifting surface,
             chord/span in [m], y = lateral centre relative to CoG [m].

    Returns
    -------
    added_inertia_roll : float [kg*m^2] for M[3,3], computed analytically by
        integrating rho*pi*c^2/4 * y^2 dy over [y - span/2, y + span/2]:
        m_a * (y^2 + span^2 / 12).
    """
    total = 0.0
    for chord, span, y in strips:
        m_a = rho * np.pi * chord**2 / 4.0 * span
        total += m_a * (y**2 + span**2 / 12.0)
    return float(total)


def strip_added_mass_heave_pitch(strips, rho: float = RHO_WATER):
    """
    2D-strip-theory hydrodynamic added mass in heave and pitch, from one or
    more lifting surfaces treated as slender strips.

    Parameters
    ----------
    strips : iterable of (chord, span, x) — one entry per lifting surface,
             chord/span in [m], x = that surface's aerodynamic-center
             position along the body x-axis relative to the CoG [m]
             (Fossen convention: positive forward).
    rho    : water density [kg/m^3]

    Returns
    -------
    (added_mass_heave, added_inertia_pitch) : added mass [kg] for M[2,2],
        added inertia [kg*m^2] for M[4,4].

    Each surface's own vertical (heave) added mass is the classical 2D
    thin-airfoil/ellipse-section result, rho*pi*c^2/4 per unit span,
    integrated over that surface's span (constant chord assumed). Pitch
    added inertia treats that same per-surface added mass as a point mass
    at its fore-aft location x and sums via the parallel-axis relation
    (sum of m_a_i * x_i^2) — a lumped approximation per lifting surface, not
    a continuous strip integral along the hull's length.
    """
    added_mass_heave = 0.0
    added_inertia_pitch = 0.0
    for chord, span, x in strips:
        m_a = rho * np.pi * chord**2 / 4.0 * span
        added_mass_heave += m_a
        added_inertia_pitch += m_a * x**2
    return float(added_mass_heave), float(added_inertia_pitch)


def strip_added_mass_sway_yaw(strips, rho: float = RHO_WATER):
    """
    2D-strip-theory hydrodynamic added mass in sway and yaw about body z, for
    one or more submerged vertical struts.

    Parameters
    ----------
    strips : iterable of (chord, wetted_span, x) — one entry per vertical strut,
             chord/wetted_span in [m], x = fore-aft position relative to CoG [m].

    Returns
    -------
    (added_mass_sway, added_inertia_yaw) : added mass [kg] for M[1,1] and added
        yaw inertia [kg*m^2] for M[5,5], using 2D flat-plate strip mass
        rho*pi*c^2/4 * wetted_span and lumped parallel-axis m_a * x^2.
    """
    added_mass_sway = 0.0
    added_inertia_yaw = 0.0
    for chord, wetted_span, x in strips:
        w = max(float(wetted_span), 0.0)
        m_a = rho * np.pi * chord**2 / 4.0 * w
        added_mass_sway += m_a
        added_inertia_yaw += m_a * x**2
    return float(added_mass_sway), float(added_inertia_yaw)


class RigidBody:
    """
    Rigid-body mass matrix and Coriolis matrix.

    Parameters
    ----------
    mass : float        Vehicle mass [kg]
    I_b  : (3,3) array  Inertia tensor about CoG in body frame [kg·m²]
    r_g  : (3,) array   CoG offset from body origin in body frame [m]
                        Usually (0, 0, 0) if body origin is at CoG.
    added_mass_heave    : diagonal M[2,2] hydrodynamic added mass [kg],
                        typically from strip_added_mass_heave_pitch()
    added_inertia_pitch : diagonal M[4,4] hydrodynamic added inertia
                        [kg*m^2], typically from strip_added_mass_heave_pitch()
    """

    def __init__(self, mass: float, I_b: np.ndarray, r_g: np.ndarray = None,
                 added_mass_heave: float = 0.0, added_inertia_pitch: float = 0.0,
                 added_inertia_roll: float = 0.0, added_mass_sway: float = 0.0,
                 added_inertia_yaw: float = 0.0):
        self.mass = mass
        self.I_b = np.asarray(I_b, dtype=float)
        self.r_g = np.zeros(3) if r_g is None else np.asarray(r_g, dtype=float)
        # Diagonal hydrodynamic added mass (sway + heave + roll + pitch + yaw). Zero by default so
        # callers that don't pass these get unchanged rigid-body-only behavior.
        self.added_mass_sway = added_mass_sway
        self.added_mass_heave = added_mass_heave
        self.added_inertia_pitch = added_inertia_pitch
        self.added_inertia_roll = added_inertia_roll
        self.added_inertia_yaw = added_inertia_yaw
        self._M = self._build_M()
        self._M_inv = np.linalg.inv(self._M)

    def _build_M(self) -> np.ndarray:
        m = self.mass
        I = self.I_b
        S_rg = skew(self.r_g)
        M = np.zeros((6, 6))
        M[:3, :3] = m * np.eye(3)
        M[:3, 3:] = -m * S_rg
        M[3:, :3] =  m * S_rg
        M[3:, 3:] = I

        # --- Hydrodynamic added mass (M_A), diagonal only ---------------------
        M[1, 1] += self.added_mass_sway
        M[2, 2] += self.added_mass_heave
        M[3, 3] += self.added_inertia_roll
        M[4, 4] += self.added_inertia_pitch
        M[5, 5] += self.added_inertia_yaw

        return M

    @property
    def M(self) -> np.ndarray:
        """6x6 rigid-body mass matrix (constant)."""
        return self._M

    def C(self, nu: np.ndarray) -> np.ndarray:
        """
        6x6 Coriolis/centripetal matrix as a function of body velocity nu.

        Parameterization from Fossen (2011) eq. 3.46.
        """
        m = self.mass
        v1 = nu[:3]   # [u, v, w]
        v2 = nu[3:]   # [p, q, r]
        S_rg = skew(self.r_g)
        C = np.zeros((6, 6))
        C[:3, 3:] = -skew(m * v1 + m * S_rg.T @ v2)
        C[3:, :3] = -skew(m * v1 + m * S_rg.T @ v2)
        C[3:, 3:] = -skew(self.I_b @ v2)

        # Added-mass Coriolis C_A(nu) (Fossen 6.43): the SAME skew structure as C_RB with
        # M_A in place of M_RB. Includes both pitch Munk moment (Z_wdot - X_udot) u w and
        # yaw Munk moment (Y_vdot - X_udot) u v, preserving exact passivity nu^T C_A nu == 0.
        a1 = np.array([0.0, self.added_mass_sway * v1[1], self.added_mass_heave * v1[2]])
        a2 = np.array([self.added_inertia_roll * v2[0], self.added_inertia_pitch * v2[1],
                       self.added_inertia_yaw * v2[2]])
        C[:3, 3:] += -skew(a1)
        C[3:, :3] += -skew(a1)
        C[3:, 3:] += -skew(a2)
        return C

    def nu_dot(self, nu: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """
        Compute body-frame acceleration given current velocity and total force.

        Parameters
        ----------
        nu  : (6,) body-frame velocity [u, v, w, p, q, r]
        tau : (6,) total generalized force [X, Y, Z, K, M, N]

        Returns
        -------
        nu_dot : (6,) body-frame acceleration
        """
        rhs = tau - self.C(nu) @ nu
        return self._M_inv @ rhs
