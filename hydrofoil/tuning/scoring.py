"""
Score a parameter set by flying it through a few standard scenarios.

Each scenario returns plain metrics (no weighting); `cost` folds them into one number for the autotuner. The
scenarios scale with the boat (wave height from the flying height, wavelength from the hull length, CG shift from
the hull length), so the same suite works for the 25 kg model and the manned preset.

    takeoff    from rest in calm water: time to foilborne, 2 cm settle time, overshoot, steady error
    head       flying start, head sea:  height rms, peak, flap rate rms
    following  flying start, following sea (the hard case for the height estimate)
    shift      flying start, CG jumps forward and to the side (a rider moving): height and roll peaks
    noisy      head sea with the noisy sensors and estimators (what the boat would really measure), so a tune
               cannot win by amplifying sensor noise into the flaps

Hard failures (hull touches down after liftoff, |roll| > 30 deg, non-finite state) set `failed`.
"""

from __future__ import annotations

import warnings

import numpy as np

SCENARIOS = ("takeoff", "head", "following", "shift", "noisy")


def _flying_state(vp):
    from hydrofoil.linearize import trim
    p = vp.params
    try:
        x, _ = trim(vp, p["target_speed"])
    except Exception:                                    # no trim at this speed: start at rest instead
        return vp.rest_state()
    return x


def run_scenario(params: dict, name: str) -> dict:
    """Fly one scenario and return its metrics."""
    warnings.simplefilter("ignore")
    from hydrofoil.vehicle import VehiclePhysics
    from hydrofoil.waves.airy import AiryWave, CalmWater
    p = dict(params)
    dt = p["dt"]
    L = p["hull_length"]
    amp = 0.3 * p["flying_height"]
    lam = 3.0 * L
    if name == "takeoff":
        T, wave, start, settle_from = 22.0, CalmWater(), "rest", 0.0
    elif name in ("head", "following", "noisy"):
        T, start, settle_from = 14.0, "flying", 3.0
        wave = AiryWave(amp, lam, direction=0.0 if name == "following" else np.pi)
        if name == "noisy":
            p["sensors"] = "noisy"
    elif name == "shift":
        T, wave, start, settle_from = 14.0, CalmWater(), "flying", 3.0
    else:
        raise ValueError(name)
    try:
        vp = VehiclePhysics(p)
    except Exception as e:                               # e.g. no trim for the LQR design
        return {"failed": True, "why": f"build: {e}"}
    s = vp.rest_state() if start == "rest" else _flying_state(vp)
    zref = p["hull_offset"] - p["flying_height"]
    t, rec = 0.0, []
    shift_at = 5.0
    lifted = False
    for i in range(int(T / dt)):
        if name == "shift" and abs(t - shift_at) < dt / 2:
            vp.set_cg_offset((0.05 * L, 0.03 * L))
        s, c, th, draft, _ = vp.step(s, t, wave)
        t += dt
        if not np.all(np.isfinite(s)):
            return {"failed": True, "why": "non-finite state"}
        lifted = lifted or draft <= 1e-6
        lim = any(v.get("limited") for v in c.get("drive", {}).values())
        rec.append((t, s[2] - zref, np.degrees(s[3]), np.degrees(s[4]), draft, c["delta_main"], c["delta_elev"],
                    c["delta_roll"], lim))
        if abs(np.degrees(s[3])) > 30.0:
            return {"failed": True, "why": "capsize (|roll| > 30 deg)"}
    r = np.array(rec, dtype=float)
    tt, dz, roll, pitch, draft, fm = r[:, 0], r[:, 1], r[:, 2], r[:, 3], r[:, 4], r[:, 5]
    out = {"failed": False}
    if name == "takeoff":
        air = np.where(draft <= 1e-6)[0]
        if not len(air):
            return {"failed": True, "why": "never foilborne"}
        i0 = air[0]
        out["t_foilborne"] = float(tt[i0])
        if draft[i0:].max() > 0:
            out.update(failed=True, why="touched down after liftoff")
        ok = np.abs(dz) < 0.02
        settle = next((tt[i] for i in range(len(tt)) if ok[i:].all()), np.nan)
        out["t_settle"] = float(settle) if np.isfinite(settle) else T + 10.0
        out["overshoot"] = float(max(0.0, -dz[i0:].min()))           # dz < 0: above the target
        out["steady_err"] = float(abs(dz[tt > T - 4].mean()))
    m = tt > settle_from
    out["z_rms"] = float(dz[m].std() if name != "takeoff" else dz[tt > T - 6].std())
    out["z_peak"] = float(np.abs(dz[m]).max())
    out["roll_peak"] = float(np.abs(roll[m]).max())
    out["pitch_peak"] = float(np.abs(pitch[m] - pitch[m].mean()).max())
    out["flap_rate_rms"] = float(np.gradient(fm[m], dt).std())
    out["sat_frac"] = float(np.mean(np.abs(fm[m]) > 19.0))
    out["limited_frac"] = float(r[m, 8].mean())
    if name != "takeoff" and draft[m].max() > 0:
        out.update(failed=True, why="hull touched the water")
    return out


def cost(metrics: dict, scale: float) -> float:
    """One number, lower is better. `scale` is the flying height [m], so height errors count relative to it."""
    c = 0.0
    for name, m in metrics.items():
        if m.get("failed"):
            c += 100.0
            continue
        c += 2.0 * m["z_rms"] / (0.05 * scale) + 0.5 * m["z_peak"] / (0.2 * scale)
        # drive effort: torque/voltage limiting means heat, wear and no authority left for the next gust, so it is
        # charged steeply (the first PID tune held the drives at their limit 63 % of the time on noisy sensors)
        c += 0.01 * m["flap_rate_rms"] + 5.0 * m["sat_frac"] + 20.0 * m["limited_frac"] + 60.0 * max(0.0, m["limited_frac"] - 0.1)
        if name == "takeoff":
            c += 0.08 * m["t_settle"] + 1.0 * m["overshoot"] / (0.1 * scale) + 4.0 * m["steady_err"] / (0.05 * scale)
        if name == "shift":
            c += 0.5 * m["roll_peak"]
    return float(c)


def score(params: dict, scenarios=SCENARIOS, pool=None) -> tuple[float, dict]:
    """(cost, {scenario: metrics}). Scenarios run in parallel when a multiprocessing pool is given."""
    jobs = [(params, n) for n in scenarios]
    res = pool.starmap(run_scenario, jobs) if pool is not None else [run_scenario(*j) for j in jobs]
    metrics = dict(zip(scenarios, res))
    return cost(metrics, params["flying_height"]), metrics
