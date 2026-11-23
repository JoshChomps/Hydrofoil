"""
Directional and irregular JONSWAP wave quantification study for the dual-flap hydrofoil.

Sweeps wave travel directions from following seas (0 deg) through beam (90 deg) to
head seas (180 deg) under matched regular Airy waves and directional JONSWAP seas,
recording encounter frequency, motion RMS/peak, split-flap activity, drive loads,
and stability/safety margins.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hydrofoil.constants import G
from hydrofoil.linearize import trim
from hydrofoil.params import MANNED_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import AiryWave, CalmWater, IrregularWave, encounter_frequency


DIRECTIONS_DEG = (0, 45, 90, 135, 180)
DIR_LABELS = {
    0: "Following (0 deg)",
    45: "Quarter-follow (45 deg)",
    90: "Beam (90 deg)",
    135: "Quarter-head (135 deg)",
    180: "Head (180 deg)",
}


def run_case(
    direction_deg: float,
    wave_type: str = "regular",
    amp: float = 0.12,
    wavelength: float = 12.0,
    duration: float = 30.0,
    spread_s: float | None = 4.0,
    noise_seed: int = 100,
    wave_seed: int = 42,
) -> dict:
    """Run one 8 m/s settled foilborne Auto+Noisy+Supervisor simulation and summarize metrics."""
    p = dict(
        MANNED_PARAMS,
        controller="auto",
        sensors="noisy",
        supervisor=True,
        allocator="jacobian",
        safety_margin="standard",
        wave_type=wave_type,
        wave_amplitude=amp,
        wave_wavelength=wavelength,
        wave_direction=direction_deg,
    )
    vp = VehiclePhysics(p)
    if vp.sensors is not None and hasattr(vp.sensors, "rng"):
        vp.sensors.rng = np.random.default_rng(noise_seed)
    s, _ = trim(vp, 8.0)
    vp.supervisor.mode = "FOILING"
    vp.supervisor.speed_ref = 8.0
    vp.supervisor.height_ref = 0.50
    vp.supervisor.pilot.update({"foil": True, "speed": 8.0, "height": 0.50, "bank": 0.0})

    dir_rad = float(np.radians(direction_deg))
    if wave_type == "irregular":
        wave = IrregularWave(
            significant_height=2.0 * amp,
            peak_wavelength=wavelength,
            direction=dir_rad,
            spread_s=spread_s,
            seed=wave_seed,
        )
    else:
        wave = AiryWave(amplitude=amp, wavelength=wavelength, direction=dir_rad)

    omega_e = encounter_frequency(float(wave.omega), 8.0, dir_rad)
    t_e = (2.0 * np.pi / abs(omega_e)) if abs(omega_e) > 1e-4 else float("inf")

    dt = float(p["dt"])
    z_ref = vp.hull_offset - 0.50
    # Warm up 2.0 s in calm flight so drives, HeaveFilter, KalmanEstimator, and LQR settle at trim
    t = 0.0
    for _ in range(int(round(2.0 / dt))):
        s, *_ = vp.step(s, t, CalmWater())
        t += dt

    n_steps = int(round(duration / dt))
    dz_err, rolls, pitches, splits, rates, lms, torqs, currs, sigs, vents = (
        [], [], [], [], [], [], [], [], [], [],
    )
    touchdowns = 0
    prev_ang = None
    for _ in range(n_steps):
        s, cm, _, dr, _ = vp.step(s, t, wave)
        t += dt
        dz_err.append((float(s[2]) - z_ref) * 100.0)
        rolls.append(float(np.degrees(s[3])))
        pitches.append(float(np.degrees(s[4])))
        dl = float(cm.get("delta_left", 0.0))
        dr_ang = float(cm.get("delta_right", 0.0))
        de = float(cm.get("delta_elev", 0.0))
        splits.append(abs(dr_ang - dl))
        cur_ang = (dl, dr_ang, de)
        if prev_ang is not None:
            rates.append(float(np.mean([abs(cur_ang[k] - prev_ang[k]) / dt for k in range(3)])))
        prev_ang = cur_ang
        al = cm.get("alloc", {})
        lms.append(float(al.get("lift_margin", 1.0)))
        drv = cm.get("drive", {})
        if drv:
            torqs.append(max(abs(float(drv[k]["M"])) for k in drv))
            currs.append(max(abs(float(drv[k]["I"])) for k in drv))
        sigs.append(float(getattr(vp, "_last_sigma", 99.0)))
        vents.append(max(float(v) for v in vp._vent.values()))
        if dr > 1e-4:
            touchdowns += 1

    def rms(arr):
        a = np.asarray(arr, dtype=float)
        return float(np.sqrt(np.mean(a * a))) if len(a) else 0.0

    return {
        "direction_deg": direction_deg,
        "label": DIR_LABELS.get(int(direction_deg), f"{direction_deg:.0f} deg"),
        "wave_type": wave_type,
        "omega_e": float(abs(omega_e)),
        "T_e": float(t_e),
        "heave_rms_cm": rms(dz_err),
        "heave_peak_cm": float(np.max(np.abs(dz_err))),
        "roll_rms_deg": rms(rolls),
        "roll_peak_deg": float(np.max(np.abs(rolls))),
        "pitch_rms_deg": rms(pitches),
        "pitch_peak_deg": float(np.max(np.abs(pitches))),
        "flap_rate_rms": rms(rates),
        "split_peak_deg": float(np.max(splits)),
        "lift_margin_min": float(np.min(lms)),
        "torque_peak_nm": float(np.max(torqs)) if torqs else 0.0,
        "current_peak_a": float(np.max(currs)) if currs else 0.0,
        "sigma_min": float(np.min(sigs)),
        "vent_max": float(np.max(vents)),
        "touchdowns": touchdowns,
        "faults": sorted(vp.supervisor.faults),
    }


def main():
    ap = argparse.ArgumentParser(description="Directional and irregular JONSWAP wave quantification study")
    ap.add_argument("--duration", type=float, default=30.0, help="simulation duration per case [s]")
    ap.add_argument("--amp", type=float, default=0.12, help="regular amplitude / half Hs [m]")
    ap.add_argument("--wavelength", type=float, default=12.0, help="peak wavelength [m]")
    args = ap.parse_args()

    reg_rows = [run_case(d, "regular", args.amp, args.wavelength, args.duration) for d in DIRECTIONS_DEG]
    irr_rows = [run_case(d, "irregular", args.amp, args.wavelength, args.duration) for d in DIRECTIONS_DEG]

    for title, rows in (
        (f"Regular Airy Wave (A={args.amp:.2f} m, lambda={args.wavelength:.1f} m, 8 m/s Auto+Noisy+Sup)", reg_rows),
        (f"Irregular Directional JONSWAP (Hs={2*args.amp:.2f} m, lambda_p={args.wavelength:.1f} m, s=4, 8 m/s)", irr_rows),
    ):
        print(f"\n### {title}")
        print("| Direction | |omega_e| [rad/s] | T_e [s] | Heave RMS / Pk [cm] | Roll RMS / Pk [deg] | Pitch RMS / Pk [deg] | Flap Rate RMS [deg/s] | Split Pk [deg] | L/W Min | Torque / I Pk | Max Vent | Faults |")
        print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|")
        for r in rows:
            te_s = f"{r['T_e']:.2f}" if np.isfinite(r["T_e"]) else "inf"
            print(
                f"| {r['label']} | {r['omega_e']:.2f} | {te_s} | "
                f"{r['heave_rms_cm']:.1f} / {r['heave_peak_cm']:.1f} | "
                f"{r['roll_rms_deg']:.2f} / {r['roll_peak_deg']:.2f} | "
                f"{r['pitch_rms_deg']:.2f} / {r['pitch_peak_deg']:.2f} | "
                f"{r['flap_rate_rms']:.1f} | {r['split_peak_deg']:.2f} | "
                f"{r['lift_margin_min']:.2f} | {r['torque_peak_nm']:.1f} Nm / {r['current_peak_a']:.1f} A | "
                f"{r['vent_max']:.2f} | {','.join(r['faults']) or 'none'} |"
            )

    print("\n### Irregular-to-Regular Peak Magnification Summary")
    print("| Direction | Heave Peak Ratio (Irr/Reg) | Roll Peak Ratio (Irr/Reg) | Split Peak Ratio (Irr/Reg) | Torque Peak Ratio |")
    print("|---|---:|---:|---:|---:|")
    for rr, ir in zip(reg_rows, irr_rows):
        h_rat = ir["heave_peak_cm"] / max(rr["heave_peak_cm"], 1e-6)
        r_rat = ir["roll_peak_deg"] / max(rr["roll_peak_deg"], 1e-6)
        s_rat = ir["split_peak_deg"] / max(rr["split_peak_deg"], 1e-6)
        t_rat = ir["torque_peak_nm"] / max(rr["torque_peak_nm"], 1e-6)
        print(f"| {rr['label']} | {h_rat:.2f}x | {r_rat:.2f}x | {s_rat:.2f}x | {t_rat:.2f}x |")


if __name__ == "__main__":
    main()
