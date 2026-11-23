"""
Hydrofoil lift and drag forces using strip theory.

Each Foil instance represents one lifting surface (main foil, elevator, rudder).
Forces are computed in the foil's local frame and returned as a 6-DOF
generalized force vector [X, Y, Z, K, M, N] in the vehicle body frame.

CL and CD tables are indexed by angle of attack (AoA) in degrees.
Load from XFLR5/XFOIL polar files, OpenFOAM, or experimental tow-tank data.
"""

from __future__ import annotations
from pathlib import Path
import numpy as np
from dataclasses import dataclass, field

from hydrofoil.constants import RHO_WATER

# Default polar: NACA 0009 from XFLR5 at Re=2.70e6, Mach=0, Ncrit=9
# Stored in data/foil_tables/naca0009_xflr5_re2700000.dat
_DAT_DIR = Path(__file__).parent.parent.parent / "data" / "foil_tables"

# Flap effectiveness d(alpha_effective)/d(delta): how much of a commanded
# flap/surface deflection actually shows up as angle-of-attack change, versus
# being absorbed by hinge-line flow losses. 1.0 = full effectiveness (an
# all-moving surface, where the whole panel rotates so commanded deflection
# literally IS angle of attack — no penalty).
MAIN_FOIL_FLAP_EFFECTIVENESS = 0.21   # plain flap, 40% chord, inboard half-span
REF_FLAP_FRAC, REF_FLAP_SPAN_FRAC = 0.4, 0.5   # the geometry the 0.21 above refers to


def main_flap_effectiveness(flap_frac: float, span_frac: float) -> float:
    """Flap effectiveness for another plain-flap geometry, scaled from the 0.21 reference by thin-airfoil
    theory: tau(c_f/c) (Glauert, hinge.thin_airfoil_coeffs) times the flapped span fraction (first order:
    a part-span flap changes the lift of its own strip). The 0.21 / (tau * span) ratio, about 0.55, stands
    for hinge-gap leakage, boundary-layer and 3-D part-span losses and is kept constant across geometries.
    Returns REF exactly at the reference geometry, so default behaviour is unchanged."""
    ff = float(np.clip(flap_frac, 0.0, 0.95))
    sf = float(np.clip(span_frac, 0.0, 1.0))
    if ff <= 0.0 or sf <= 0.0:
        return 0.0
    from hydrofoil.actuation.hinge import thin_airfoil_coeffs
    tau = thin_airfoil_coeffs(ff)["tau"]
    tau0 = thin_airfoil_coeffs(REF_FLAP_FRAC)["tau"]
    return MAIN_FOIL_FLAP_EFFECTIVENESS * (tau * sf) / (tau0 * REF_FLAP_SPAN_FRAC)
ELEVATOR_FLAP_EFFECTIVENESS = 1.0     # all-moving surface (whole elevator rotates)

# Oswald span-efficiency factor e, used in the induced-drag term CL^2/(pi*AR*e).
# 0.80 reflects the main foil being split into two independent halves at the
# centerline (no continuous circulation across the root) rather than one
# uninterrupted wing, which would allow a higher e.
MAIN_FOIL_OSWALD_EFFICIENCY = 0.80


@dataclass
class FoilGeometry:
    """Physical dimensions of a single lifting surface."""
    chord: float        # mean chord length [m]
    span: float         # span (one side) [m]
    aoa_offset: float = 0.0  # fixed geometric AoA offset (washout, camber) [deg]

    @property
    def area(self) -> float:
        return self.chord * self.span


class FoilTables:
    """
    Lookup tables mapping AoA (degrees) to CL and CD.

    Interpolates linearly within the polar range. AoA values outside the
    table are clamped to the table boundary — this avoids extrapolation
    artifacts but means the model does not capture post-stall lift recovery.
    Add experimental data beyond stall when available.

    Attributes
    ----------
    aoa_deg : (N,) sorted array of AoA breakpoints [deg]
    CL      : (N,) lift coefficient at each breakpoint
    CD      : (N,) drag coefficient at each breakpoint
    source  : human-readable label (foil name + Re + source)
    """

    def __init__(
        self,
        aoa_deg: np.ndarray,
        CL: np.ndarray,
        CD: np.ndarray,
        source: str = "unknown",
    ):
        self.aoa_deg = np.asarray(aoa_deg, dtype=float)
        self.CL = np.asarray(CL, dtype=float)
        self.CD = np.asarray(CD, dtype=float)
        self.source = source
        self.aoa_min = self.aoa_deg[0]
        self.aoa_max = self.aoa_deg[-1]

    def cl(self, aoa_deg: float) -> float:
        a = self.aoa_min if aoa_deg < self.aoa_min else (self.aoa_max if aoa_deg > self.aoa_max else float(aoa_deg))
        return float(np.interp(a, self.aoa_deg, self.CL))

    def cd(self, aoa_deg: float) -> float:
        a = self.aoa_min if aoa_deg < self.aoa_min else (self.aoa_max if aoa_deg > self.aoa_max else float(aoa_deg))
        return float(np.interp(a, self.aoa_deg, self.CD))

    @property
    def cl_slope_per_deg(self) -> float:
        """Lift slope dCL/dα near zero AoA [per degree]."""
        mask = np.abs(self.aoa_deg) <= 3.0
        if mask.sum() < 2:
            return float(np.gradient(self.CL, self.aoa_deg).mean())
        return float(np.polyfit(self.aoa_deg[mask], self.CL[mask], 1)[0])

    @classmethod
    def from_dat(cls, path: Path | str) -> "FoilTables":
        """
        Load a polar from a plain-text file with columns: alpha  CL  CD.
        Lines starting with '#' are treated as comments.
        """
        path = Path(path)
        data = []
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    data.append([float(parts[0]), float(parts[1]), float(parts[2])])
        arr = np.array(data)
        return cls(arr[:, 0], arr[:, 1], arr[:, 2], source=path.stem)

    @classmethod
    def naca0009(cls) -> "FoilTables":
        """NACA 0009 polar from XFLR5, Re=2.70e6, Mach=0."""
        return cls.from_dat(_DAT_DIR / "naca0009_xflr5_re2700000.dat")


def free_surface_lift_factor(submergence_over_chord: float) -> float:
    """
    Empirical free-surface lift-loss factor for a fully-submerged foil
    operating at shallow depth, as a function of submergence-to-chord ratio
    h/c (foil depth below the free surface, divided by chord).

    A foil close to the free surface loses lift (the surface behaves like an
    image system that partially cancels the foil's bound circulation); the
    loss shrinks as the foil goes deeper. This is a PLACEHOLDER monotonic fit
    — not derived from free-surface panel-method or experimental data —
    tuned only to land near ~0.85 at h/c=3 (this vehicle's nominal flying
    submergence) and approach 1.0 (no loss) by h/c=10. Replace with a
    validated correction once available.
    """
    h_c = max(submergence_over_chord, 0.0)
    DECAY_LENGTH_CHORDS = 1.58   # fit: factor(3) ~= 0.85, factor(10) ~= 0.998
    return float(1.0 - np.exp(-h_c / DECAY_LENGTH_CHORDS))


def _cross(a, b):
    """3-vector cross product (np.cross has ~10x the overhead for tiny vectors; this runs ~400x per step)."""
    return np.array([a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]])


class Foil:
    """
    A single lifting surface attached to the vehicle.

    Parameters
    ----------
    geometry    : FoilGeometry
    tables      : FoilTables
    r_attach    : (3,) position of foil aerodynamic center in body frame [m]
                  e.g. r_attach = [1.2, 0.0, -0.3] means 1.2 m forward,
                  0.3 m below body origin
    lift_axis   : (3,) unit vector in body frame along which lift acts
                  For a horizontal foil: [0, 0, -1] (lift = upward = -z_body)
    drag_axis   : (3,) unit vector along which drag acts (opposing motion)
                  Typically derived from flow direction, not set directly.
    rho         : water density [kg/m³]
    flap_effectiveness : d(alpha_eff)/d(delta) — see module-level
                  MAIN_FOIL_FLAP_EFFECTIVENESS / ELEVATOR_FLAP_EFFECTIVENESS
    oswald_efficiency  : Oswald span-efficiency e, used in induced drag
    """

    def __init__(
        self,
        geometry: FoilGeometry,
        tables: FoilTables,
        r_attach: np.ndarray,
        lift_axis: np.ndarray = np.array([0.0, 0.0, -1.0]),
        rho: float = RHO_WATER,
        flap_effectiveness: float = 1.0,
        oswald_efficiency: float = 0.80,
        normal_axis: np.ndarray | None = None,
        flap_cm_delta: float = 0.0,
        flap_stall_deg: float | None = None,
    ):
        self.geometry = geometry
        self.tables = tables
        self.r_attach = np.asarray(r_attach, dtype=float)
        # normal_axis: body-frame unit vector the foil's AoA is measured along (positive
        # AoA = flow arriving from the -normal side... i.e. velocity component ALONG normal).
        # Horizontal foil: (0,0,1) (body z, down). Vertical strut: (0,1,0) (starboard), so the
        # strut's AoA is the side-slip angle and it develops a side force. lift_axis = -normal.
        if normal_axis is not None:
            n = np.asarray(normal_axis, dtype=float)
            nn = float(np.linalg.norm(n))
            if nn <= 1e-9:
                raise ValueError("normal_axis must be a non-zero 3-vector")
            self.normal = n / nn
            self.lift_axis = -self.normal
        else:
            la = np.asarray(lift_axis, dtype=float)
            ln = float(np.linalg.norm(la))
            if ln <= 1e-9:
                raise ValueError("lift_axis must be a non-zero 3-vector")
            self.lift_axis = la / ln
            self.normal = -self.lift_axis
        self.rho = rho
        # Default (1.0) is a no-penalty, all-moving-surface assumption, so
        # callers that don't pass this explicitly (e.g. sim.py) keep their
        # prior behavior unchanged. See MAIN_FOIL_FLAP_EFFECTIVENESS /
        # ELEVATOR_FLAP_EFFECTIVENESS above for this vehicle's real values.
        self.flap_effectiveness = flap_effectiveness
        self.oswald_efficiency = max(float(oswald_efficiency), 1e-4)
        # Section pitching-moment slope of the flap about the quarter chord [per rad of REAL flap
        # deflection, nose-up positive, already scaled by flap span fraction]. A flap-down force
        # increment acts aft of c/4, so it also adds a nose-down couple. 0 = none (all-moving foil).
        self.flap_cm_delta = flap_cm_delta
        self.flap_stall_deg = flap_stall_deg
        self.last_CL = 0.0   # corrected CL of the latest generalized_force call (read by the tail downwash)
        # Aspect ratio of this panel from its own span/chord (rectangular
        # planform assumed). Each main-foil half is modeled as its own Foil
        # at its own half-span, so this is that half-panel's local AR, not
        # the un-split wing's.
        self.aspect_ratio = (float(geometry.span) / max(float(geometry.chord), 1e-4)
                             if geometry.chord > 0 and geometry.span > 0 else 0.0)
        self._a2d = tables.cl_slope_per_deg * 180.0 / np.pi   # section slope, per rad
        # Finite-wing lift-curve correction, Prandtl/Helmbold form with the SAME
        # Oswald efficiency used for induced drag:
        #     a_3D = a_2D / (1 + a_2D / (pi * e * AR))
        # a_2D is the polar's own section slope (per rad), not a hard-coded 2*pi.
        # (Previously lift used AR/(AR+2), an e = 1 form, while induced drag used
        # e = 0.80 -- the two corrections disagreed about the same wing.)
        self.lift_slope_factor = self._lift_slope_factor(self.aspect_ratio)

    def _lift_slope_factor(self, ar: float) -> float:
        if ar <= 1e-6:
            return 0.0
        return 1.0 / (1.0 + self._a2d / (np.pi * self.oswald_efficiency * ar))

    def flow(self, nu: np.ndarray, r_attach: np.ndarray | None = None) -> tuple[float, float]:
        """In-plane flow speed [m/s] and geometric angle of attack [rad, excl. flap] at the foil
        (same kinematics as generalized_force; used by the actuator load model)."""
        r = self.r_attach if r_attach is None else r_attach
        V_foil = nu[:3] + _cross(nu[3:], r)
        V_n = float(V_foil @ self.normal)
        return float(np.hypot(V_foil[0], V_n)), float(np.arctan2(V_n, V_foil[0]) + np.radians(self.geometry.aoa_offset))

    def generalized_force(
        self,
        nu: np.ndarray,
        delta: float = 0.0,
        submergence_over_chord: float | None = None,
        span: float | None = None,
        r_attach: np.ndarray | None = None,
        downwash_rad: float = 0.0,
        ventilation: float = 0.0,
    ) -> np.ndarray:
        """
        Compute 6-DOF generalized force contribution of this foil.

        Parameters
        ----------
        nu    : (6,) body-frame velocity [u, v, w, p, q, r] [m/s, rad/s]
        delta : flap/servo deflection angle commanded, BEFORE flap
                effectiveness is applied [deg]
        submergence_over_chord : this foil's depth below the free surface
                divided by its chord (h/c). None (default) skips the
                free-surface correction entirely (factor == 1.0) — callers
                that don't track submergence (e.g. sim.py) get unchanged
                behavior.

        downwash_rad : downwash angle at this foil from an upstream wing [rad]; reduces the AoA.

        span, r_attach : optional per-call overrides of the geometric span and the
                aerodynamic-centre position (used for struts whose wetted length
                changes with depth). Defaults: the constructor values.

        Returns
        -------
        tau : (6,) [X, Y, Z, K, M, N] in body frame [N, N, N, Nm, Nm, Nm]
        """
        span = self.geometry.span if span is None else span
        r_attach = self.r_attach if r_attach is None else np.asarray(r_attach, dtype=float)
        if span <= 1e-3 or self.geometry.chord <= 1e-4:
            self.last_CL = 0.0
            return np.zeros(6)
        ar = span / self.geometry.chord
        V_body = nu[:3]   # translational velocity at body origin
        omega = nu[3:]    # angular velocity

        # velocity at foil attachment point (body frame)
        V_foil = V_body + _cross(omega, r_attach)
        # Only the chordwise (x) and normal (z) velocity components load a
        # horizontal foil. Spanwise (y) flow produces neither lift nor dynamic
        # pressure on it, so it is excluded (previously |V| included it, which
        # over-predicted force in side-slip).
        V_n = float(V_foil @ self.normal)
        V_plane = np.array([V_foil[0], 0.0, 0.0]) + V_n * self.normal
        speed = np.linalg.norm(V_plane)

        if speed < 0.01:
            self.last_CL = 0.0
            return np.zeros(6)

        # flow direction (opposing velocity = drag direction), in the x-z plane
        V_hat = V_plane / speed

        # AoA = arctan(w / u) in NED body frame.
        # Plain flaps begin trailing-edge separation above flap_stall_deg (~15 deg): roll off
        # incremental lift effectiveness smoothly and add separation drag while leaving normal
        # trim deflections completely linear.
        eff_delta = delta
        cd_flap_sep = 0.0
        if self.flap_stall_deg is not None and abs(delta) > self.flap_stall_deg:
            d0 = self.flap_stall_deg
            exc = abs(delta) - d0
            eff_delta = (d0 + exc - 0.55 * (exc * exc) / (exc + 4.0)) * (1.0 if delta >= 0.0 else -1.0)
            cd_flap_sep = 0.00025 * exc * exc

        aoa_rad = np.arctan2(V_n, V_foil[0])
        aoa_deg = np.degrees(aoa_rad) + self.geometry.aoa_offset + self.flap_effectiveness * eff_delta - np.degrees(downwash_rad)
        aoa_deg = ((aoa_deg + 180.0) % 360.0) - 180.0

        q = 0.5 * self.rho * speed**2
        A = self.geometry.chord * span

        # 1. 2D section polar (XFLR5) within table bounds; smooth flat-plate post-stall beyond
        # so high-AoA and reverse flow (V_foil[0] < 0) go continuously to CL = 0 at +-180 deg.
        if self.tables.aoa_min <= aoa_deg <= self.tables.aoa_max:
            CL_2d = self.tables.cl(aoa_deg)
            CD_2d = self.tables.cd(aoa_deg)
        else:
            edge = self.tables.aoa_max if aoa_deg > 0.0 else self.tables.aoa_min
            cl_edge = self.tables.cl(edge)
            cd_edge = self.tables.cd(edge)
            excess = abs(aoa_deg - edge)
            blend = min(1.0, excess / 7.0)
            blend = blend * blend * (3.0 - 2.0 * blend)
            a_rad = np.radians(aoa_deg)
            cl_post = 1.15 * abs(np.sin(2.0 * a_rad)) * (1.0 if a_rad >= 0.0 else -1.0)
            cd_post = 2.0 * np.sin(a_rad) ** 2 + 0.02
            CL_2d = (1.0 - blend) * cl_edge + blend * cl_post
            CD_2d = (1.0 - blend) * cd_edge + blend * cd_post
        CD_2d += cd_flap_sep

        # 2. Finite-span correction (see __init__): strip theory here uses ONE
        # strip per foil, so the 2D section CL overestimates real 3D lift.
        CL = CL_2d * self._lift_slope_factor(ar)

        # 3. Free-surface lift loss, emergence wetting, and breach ventilation hysteresis.
        wet = 1.0
        if submergence_over_chord is not None:
            if submergence_over_chord <= 0.0:
                self.last_CL = 0.0
                return np.zeros(6)
            if submergence_over_chord < 0.25:
                t_w = submergence_over_chord / 0.25
                wet = t_w * t_w * (3.0 - 2.0 * t_w)
            CL *= free_surface_lift_factor(submergence_over_chord)
        vent = min(1.0, max(0.0, float(ventilation)))
        if vent > 0.0:
            CL *= (1.0 - 0.55 * vent)

        self.last_CL = float(CL)

        # 4. Induced drag from the corrected (3D, free-surface-adjusted) CL,
        # added on top of wetted 2D profile drag + any ventilated cavity drag.
        CD_induced = CL**2 / max(np.pi * ar * self.oswald_efficiency, 1e-6)
        CD = (CD_2d + (0.08 * vent if vent > 0.0 else 0.0)) * wet + CD_induced

        L_mag = q * CL * A
        D_mag = q * CD * A

        # lift perpendicular to flow in the lift plane
        # V_hat x (a x V_hat) = a - (a.V_hat) V_hat for unit V_hat: the part of the lift axis
        # perpendicular to the flow.
        lift_dir = self.lift_axis - (self.lift_axis @ V_hat) * V_hat
        ln = np.linalg.norm(lift_dir)
        if ln > 1e-9:
            lift_dir = lift_dir / ln

        force = L_mag * lift_dir - D_mag * V_hat   # [N] in body frame
        moment = _cross(r_attach, force)     # [Nm] about body origin

        tau = np.zeros(6)
        tau[:3] = force
        tau[3:] = moment
        if self.flap_cm_delta:
            tau[4] += q * A * self.geometry.chord * self.flap_cm_delta * np.radians(delta) * wet * (1.0 - 0.55 * vent)
        return tau
