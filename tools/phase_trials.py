"""
Per-phase controller trials (docs/21): which controller should fly each phase of a supervised mission?

Phases (control/supervisor.py modes) and the choices tried (params["phase_controllers"]):
    DISPLACEMENT  fixed (all surfaces at their displacement angle: no stability system)
                  pid   (main flap fixed; PID damps pitch and roll on the elevator and differential flap)
                  lqr   (main flap fixed; LQR on pitch and roll)
    TAKEOFF       pid   (PID throughout the takeoff and climb)
                  auto  (PID until the hull is clear and the height error small, then a blended LQR handover)
                  lqr   (LQR from the takeoff request, hull still in the water)
    FOILING       pid | lqr
    LANDING       pid | lqr     (switching back to the PID fades over the LQR's blend time)
Thrust is always the surge PID.

Mission (manned preset): 10 s displacement at 3 m/s; foil request at 7 m/s; 8.5 m/s at 45 s; 6.5 m/s at 65 s;
land at 85 s; 30 s more in displacement. Calm, head and following sea (0.15 m, 10.8 m), noisy sensors by default.

    python tools/phase_trials.py                      # every permutation x 3 seas -> results JSON lines + table
    python tools/phase_trials.py --only "pid,auto,lqr,lqr" --seas calm
"""

from __future__ import annotations

import argparse
import itertools
import json
import multiprocessing as mp
import sys
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

PHASES = ("DISPLACEMENT", "TAKEOFF", "FOILING", "LANDING")
CHOICES = {"DISPLACEMENT": ("fixed", "pid", "lqr"), "TAKEOFF": ("pid", "auto", "lqr"),
           "FOILING": ("pid", "lqr"), "LANDING": ("pid", "lqr")}
PLAN = ((0.0, {"speed": 3.0}), (10.0, {"foil": True, "speed": 7.0}), (45.0, {"speed": 8.5}),
        (65.0, {"speed": 6.5}), (85.0, {"foil": False, "speed": 3.0}))
T_END = 115.0
SEAS = {"calm": None, "head": np.pi, "following": 0.0}
G = 9.81


def _wave(p, sea):
    from hydrofoil.waves.airy import AiryWave, CalmWater
    if SEAS[sea] is None:
        return CalmWater()
    return AiryWave(0.3 * p["flying_height"], 3.0 * p["hull_length"], direction=SEAS[sea])


def mission(combo: tuple, sea: str, sensors: str = "noisy", seed: int = 0) -> dict:
    """Fly the mission with one controller per phase; return per-phase metrics."""
    warnings.simplefilter("ignore")
    from hydrofoil.params import MANNED_PARAMS
    from hydrofoil.vehicle import VehiclePhysics
    pc = dict(zip(PHASES, combo))
    p = dict(MANNED_PARAMS, controller="lqr", supervisor=True, sensors=sensors, sensor_seed=seed,
             phase_controllers=pc)
    vp = VehiclePhysics(p)
    sup, dt = vp.supervisor, p["dt"]
    wave = _wave(p, sea)
    s, t = vp.rest_state(), 0.0
    rows = []
    fail = None
    w_prev = 0.0
    for i in range(int(T_END / dt)):
        for tt, cmd in PLAN:
            if i == int(round(tt / dt)):
                sup.pilot.update(cmd)
        mode = sup.mode
        s, c, thrust, draft, wave_elev = vp.step(s, t, wave)
        t += dt
        if not np.all(np.isfinite(s)):
            fail = "non-finite state"; break
        if abs(np.degrees(s[3])) > 30.0:
            fail = "capsize"; break
        u = float(s[6])
        vz = float(-s[6] * np.sin(s[4]) + s[7] * np.cos(s[4]) * np.sin(s[3]) + s[8] * np.cos(s[4]) * np.cos(s[3]))     # NED down-velocity: + = sinking
        az = (vz - w_prev) / dt
        w_prev = vz
        dr = c.get("drive", {})
        p_drv = sum(d["P"] for d in dr.values())
        lim = any(d.get("limited") for d in dr.values())
        i_pk = max((abs(d["I"]) for d in dr.values()), default=0.0)
        clear = (p["hull_offset"] - s[2]) - wave_elev                           # hull clearance over the local surface
        zerr = (p["hull_offset"] - s[2]) - p["flying_height"]                   # against the supervisor's reference
        rows.append((t, PHASES.index(mode) if mode in PHASES else 4, u, clear, zerr, np.degrees(s[3]),
                     np.degrees(s[4]), draft > 1e-4, vz, az, thrust * u, p_drv, lim, i_pk, c["delta_main"]))
    out = {"combo": list(combo), "sea": sea, "sensors": sensors, "failed": fail,
           "log": [(tt, a, b, w) for tt, a, b, w in sup.log], "faults": sorted(sup.faults)}
    if not rows:
        return out
    r = np.array(rows, dtype=float)
    tt, ph, u, clear, zerr, roll, pitch, wet, vz, az, p_prop, p_drv, lim, ipk, fm = r.T
    if any(to == "EMERGENCY_LANDING" for _t, _f, to, _w in sup.log):
        out["failed"] = out["failed"] or "emergency landing: " + ",".join(sorted(sup.faults))
    if any(frm == "TAKEOFF" and to not in ("FOILING",) for _t, frm, to, _w in sup.log):
        out["failed"] = out["failed"] or "takeoff aborted"

    def seg(code, first=True):
        idx = np.where(ph == code)[0]
        if not len(idx):
            return idx
        if first:                                               # the first contiguous block
            br = np.where(np.diff(idx) > 1)[0]
            idx = idx[: br[0] + 1] if len(br) else idx
        return idx

    def energy_wh(idx):
        return float((p_prop[idx].sum() + p_drv[idx].sum()) * dt / 3600.0) if len(idx) else 0.0

    d = seg(0)
    out["DISPLACEMENT"] = {"roll_rms": float(np.std(roll[d])), "pitch_rms": float(np.std(pitch[d])),
                           "roll_peak": float(np.max(np.abs(roll[d]))), "energy_wh": energy_wh(d)} if len(d) else {}
    k = seg(1)
    if len(k):
        wet_k = wet[k].astype(bool)
        lifted = np.where(~wet_k)[0]
        touches = int(np.sum(np.diff(wet_k.astype(int)) > 0)) if len(lifted) else 0
        out["TAKEOFF"] = {"duration": float(tt[k[-1]] - tt[k[0]]), "energy_wh": energy_wh(k),
                          "touches_after_liftoff": touches, "pitch_peak": float(np.max(np.abs(pitch[k]))),
                          "roll_peak": float(np.max(np.abs(roll[k]))), "limited_frac": float(lim[k].mean()),
                          "i_peak": float(ipk[k].max()), "flap_sat_frac": float(np.mean(np.abs(fm[k]) > 19.0))}
    f = seg(2)
    if len(f):
        f2 = f[tt[f] > tt[f[0]] + 3.0]                          # after the climb has settled
        dist_km = float(u[f].sum() * dt / 1000.0)
        out["FOILING"] = {"duration": float(tt[f[-1]] - tt[f[0]]), "energy_wh": energy_wh(f),
                          "wh_per_km": energy_wh(f) / max(dist_km, 1e-6),
                          "z_rms_mm": float(np.sqrt(np.mean(zerr[f2] ** 2)) * 1e3) if len(f2) else float("nan"),
                          "clear_min": float(clear[f].min()), "hull_contacts": int(np.sum(np.diff(wet[f].astype(int)) > 0)),
                          "roll_peak": float(np.max(np.abs(roll[f]))), "pitch_peak": float(np.max(np.abs(pitch[f2]))) if len(f2) else 0.0,
                          "az_rms_g": float(np.std(az[f2]) / G) if len(f2) else float("nan"),
                          "flap_rate_rms": float(np.std(np.gradient(fm[f], dt))), "limited_frac": float(lim[f].mean()),
                          "i_peak": float(ipk[f].max())}
    l_ = seg(3)
    if len(l_):
        after = l_[-1] + 1
        tail = np.arange(after, min(after + int(10.0 / dt), len(tt)))        # 10 s after "hull down": slaps
        both = np.concatenate([l_, tail])
        first = np.where(wet[both].astype(bool))[0]                           # the mode flips on the contact step
        contacts = int(np.sum(np.diff(np.concatenate([wet[l_], wet[tail]]).astype(int)) > 0))
        out["LANDING"] = {"duration": float(tt[l_[-1]] - tt[l_[0]]), "energy_wh": energy_wh(l_),
                          "sink_at_contact": float(vz[both[first[0]]]) if len(first) else float("nan"),
                          "contacts_incl_10s_after": contacts,
                          "pitch_peak": float(np.max(np.abs(pitch[np.concatenate([l_, tail])]))),
                          "roll_peak": float(np.max(np.abs(roll[np.concatenate([l_, tail])]))),
                          "vz_max_after": float(vz[tail].max()) if len(tail) else float("nan")}
    out["completed"] = bool(out["failed"] is None and sup.mode == "DISPLACEMENT"
                            and any(to == "FOILING" for _t, _f, to, _w in sup.log))
    out["energy_total_wh"] = float((p_prop.sum() + p_drv.sum()) * dt / 3600.0)
    out["drive_energy_wh"] = float(p_drv.sum() * dt / 3600.0)
    out["i_peak"] = float(ipk.max())
    out["mission_time_to_foil"] = next((tt_ - PLAN[1][0] for tt_, _f, to, _w in sup.log if to == "FOILING"), None)
    return out


def _job(a):
    combo, sea, sensors = a
    try:
        return mission(combo, sea, sensors)
    except Exception as e:                                      # report, do not lose the batch
        return {"combo": list(combo), "sea": sea, "sensors": sensors, "failed": f"error: {e!r}"}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seas", default="calm,head,following")
    ap.add_argument("--sensors", default="noisy", choices=("noisy", "ideal"))
    ap.add_argument("--only", help="one combo, e.g. 'pid,auto,lqr,lqr' (DISPLACEMENT,TAKEOFF,FOILING,LANDING)")
    ap.add_argument("--jobs", type=int, default=max(1, (mp.cpu_count() or 2) - 1))
    ap.add_argument("--out", type=Path, default=Path("phase_trials.jsonl"))
    a = ap.parse_args()
    combos = [tuple(a.only.split(","))] if a.only else list(itertools.product(*(CHOICES[k] for k in PHASES)))
    jobs = [(c, sea, a.sensors) for c in combos for sea in a.seas.split(",")]
    with mp.get_context("spawn").Pool(a.jobs) as pool, a.out.open("w") as fh:
        for r in pool.imap_unordered(_job, jobs):
            fh.write(json.dumps(r, default=float) + "\n"); fh.flush()
            print(r["sea"], ",".join(r["combo"]), "FAILED " + str(r["failed"]) if r.get("failed") else "ok", flush=True)


if __name__ == "__main__":
    main()
