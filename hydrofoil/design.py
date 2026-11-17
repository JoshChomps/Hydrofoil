"""
Boat design layer for the editor and the autotuner: turns design edits into a consistent parameter set, and works
out what they do (takeoff speed, cruise trim, power, margins) without flying the boat.

Edits (`overrides`, all optional; anything else that is a params key is copied through):
    payloads        list of {"name", "mass" [kg], "x" [m fwd of the nominal CG], "y" [m to starboard], "z" [m up]}.
                    They replace the preset's own `default_payloads` (the manned preset carries a 75 kg rider);
                    the empty boat is mass - sum(default payloads). Total mass, CG offset (x, y) and inertia follow.
                    The CG height is not modelled (the sim has no vertical CG offset) and is reported only.
    strut_length    hull bottom to main foil [m]; moves both foils (elevator stays 0.1 m higher)
    main_span       per half [m]; the halves stay 0.1 m apart at the root
    main_chord, main_incidence_deg, main_flap_frac, elev_chord, elev_span, elev_pos_x, flying_height, target_speed ...

When anything that changes trim is edited, the trim feed-forward and the LQR / estimator speed schedule are
re-derived from the new boat (the speeds where it can trim with the flap inside +-15 deg).

Foil "shape" is limited by the model: rectangular strip foils with one section polar (NACA 0009), so span, chord,
aspect ratio, flap fraction and incidence are what can change; taper, sweep, twist and section are not modelled.
"""

from __future__ import annotations

import warnings

import numpy as np

from hydrofoil.constants import RHO_WATER, G

TRIM_EDIT_KEYS = {"payloads", "mass", "strut_length", "main_span", "main_chord", "main_incidence_deg", "main_flap_frac",
                  "main_flap_span_frac", "main_flap_setback", "main_flap_effectiveness",
                  "elev_chord", "elev_span", "elev_pos_x", "main_pos_x", "flying_height", "hull_offset", "cg_offset"}
FLAP_TRIM_LIMIT = 15.0                                    # deg of +-20: margin kept for waves and control


def _point_inertia(m, r):
    r = np.asarray(r, float)
    return m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))


def mass_properties(p: dict, payloads: list | None) -> dict:
    """Total mass, CG (relative to the nominal CG) and inertia about the new CG for the given payload list."""
    defaults = p.get("default_payloads", [])
    m_def = sum(q["mass"] for q in defaults)
    m_empty = p["mass"] - m_def
    I_nom = np.diag([p["ixx"], p["iyy"], p["izz"]])                   # boat + default payloads, about the nominal CG
    I_empty = I_nom - sum((_point_inertia(q["mass"], (q["x"], q["y"], q["z"])) for q in defaults), np.zeros((3, 3)))
    loads = defaults if payloads is None else payloads
    # the empty boat's own CG: nominal CG = (m_empty r_e + sum m_i r_i) / m_total = 0
    r_e = -sum((q["mass"] * np.array([q["x"], q["y"], q["z"]]) for q in defaults), np.zeros(3)) / max(m_empty, 1e-9)
    m = m_empty + sum(q["mass"] for q in loads)
    cg = (m_empty * r_e + sum((q["mass"] * np.array([q["x"], q["y"], q["z"]]) for q in loads), np.zeros(3))) / m
    # I_empty is about the nominal CG; move everything to the new CG
    I_e_cm = I_empty - _point_inertia(m_empty, r_e)                    # empty boat about its own CG
    I = I_e_cm + _point_inertia(m_empty, r_e - cg)
    for q in loads:
        I = I + _point_inertia(q["mass"], np.array([q["x"], q["y"], q["z"]]) - cg)
    return {"mass": float(m), "mass_empty": float(m_empty), "cg": cg.tolist(), "inertia": np.diag(I).tolist()}


def apply_design(p: dict, overrides: dict) -> dict:
    """New params dict with the edits applied (the input is not modified)."""
    p = dict(p)
    ov = dict(overrides or {})
    payloads = ov.pop("payloads", None)
    strut = ov.pop("strut_length", None)
    for k, v in ov.items():
        if k in p or k in ("lqr_weights",):
            p[k] = v
    if "main_span" in ov:
        p["main_lateral_offset"] = 0.05 + p["main_span"] / 2
    if strut is not None:
        dz = p["main_pos_z"] - p["elev_pos_z"]
        p["main_pos_z"] = float(strut) - p["hull_offset"]
        p["elev_pos_z"] = p["main_pos_z"] - dz
    if payloads is not None:
        mp_ = mass_properties(p, payloads)
        p["mass"] = mp_["mass"]
        p["ixx"], p["iyy"], p["izz"] = mp_["inertia"]
        p["cg_offset"] = (mp_["cg"][0], mp_["cg"][1])
        p["default_payloads"] = payloads
    if overrides and (TRIM_EDIT_KEYS & set(overrides)):
        _reschedule(p)
    return p


def _trim_flap(p: dict, speed: float):
    from hydrofoil.linearize import trim
    from hydrofoil.vehicle import VehiclePhysics
    vp = VehiclePhysics(dict(p, controller="pid", sensors="ideal", supervisor=False))   # at the actual CG
    try:
        _, u = trim(vp, speed, u0=(5.0, 0.0, 0.0, 200.0), max_iter=80)
        return float(u[0]), float(u[1]), float(u[3])
    except Exception:
        return None


_CURVES: dict = {}
_CONTROL_KEYS = ("lqr_", "est_", "trim_ff", "kp_", "ki_", "kd_", "controller", "sensors", "supervisor", "kf_",
                 "servo_params", "default_payloads", "target_speed", "max_thrust", "phi_ref", "theta_ref")


def trim_curve(p: dict, speeds=None) -> list[dict]:
    """Level trim at each speed (cached on the physical parameters only: controller settings do not change trim)."""
    warnings.simplefilter("ignore")
    speeds = speeds if speeds is not None else np.round(np.arange(3.0, 12.01, 0.5), 2)
    key = (repr(sorted((k, repr(v)) for k, v in p.items() if not k.startswith(_CONTROL_KEYS))), tuple(speeds))
    if key in _CURVES:
        return _CURVES[key]
    out = []
    for s in speeds:
        r = _trim_flap(p, float(s))
        out.append({"speed": float(s), "flap": r and r[0], "elev": r and r[1], "thrust": r and r[2]})
    if len(_CURVES) > 64:
        _CURVES.clear()
    _CURVES[key] = out
    return out


def _reschedule(p: dict):
    """Re-derive the trim feed-forward and the LQR / estimator speed schedule for an edited boat."""
    curve = trim_curve(p)
    ok = [c for c in curve if c["flap"] is not None and abs(c["flap"]) <= FLAP_TRIM_LIMIT]
    if not ok:
        return
    v_lo = ok[0]["speed"]
    v_hi = max(c["speed"] for c in ok)
    ref = min(ok, key=lambda c: abs(c["speed"] - max(v_lo + 1.0, 0.75 * p["target_speed"])))
    p["trim_ff_deg"], p["trim_ff_speed"] = ref["flap"], ref["speed"]
    hi = min(v_hi, max(p["target_speed"] * 1.2, v_lo + 3.0))
    p["lqr_speeds"] = tuple(float(x) for x in np.round(np.linspace(v_lo, hi, 5), 2))
    p["lqr_engage_speed"] = float(v_lo + 0.3)
    p["est_engage_speed"] = float(v_lo)


def derived(p: dict) -> dict:
    """What the design does, without flying it: wing loading, takeoff and cruise trim, power, margins."""
    warnings.simplefilter("ignore")
    S = 2 * p["main_chord"] * p["main_span"]
    W = p["mass"] * G
    curve = trim_curve(p)
    ok = [c for c in curve if c["flap"] is not None and abs(c["flap"]) <= FLAP_TRIM_LIMIT]
    cruise = _trim_flap(p, p["target_speed"])
    depth = (p["hull_offset"] - p["flying_height"]) + p["main_pos_z"]
    q = 0.5 * RHO_WATER * p["target_speed"] ** 2
    cg = p.get("cg_offset", (0.0, 0.0))
    return {
        "mass": p["mass"], "cg_x": cg[0], "cg_y": cg[1],
        "wing_area": S, "aspect_ratio": p["main_span"] / p["main_chord"], "wing_loading": W / S,
        "foil_depth": depth, "strut_length": p["hull_offset"] + p["main_pos_z"],
        "v_min_foil": ok[0]["speed"] if ok else None, "v_max_trim": max(c["speed"] for c in ok) if ok else None,
        "cruise_speed": p["target_speed"],
        "cruise_flap": cruise and cruise[0], "cruise_elev": cruise and cruise[1],
        "cruise_thrust": cruise and cruise[2], "cruise_power_kw": cruise and cruise[2] * p["target_speed"] / 1000,
        "flap_margin": cruise and 20.0 - abs(cruise[0]),
        "cavitation_number": (101325 + RHO_WATER * G * depth - 2340) / q,
        "reynolds": p["target_speed"] * p["main_chord"] / 1.05e-6,
        "curve": curve,
    }
