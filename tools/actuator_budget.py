"""
Actuator budget: fly the closed-loop sim through a set of scenarios, turn each physical
control-surface trajectory into hinge torque -> motor torque -> current / voltage / bus power,
and check it against the moteus r4.11 + motor + gearbox limits, for a sweep of gear ratios.

    python tools/actuator_budget.py                 # all scenarios, N = 20 detail + ratio sweep
    python tools/actuator_budget.py --ratios 10 20 30 --scenarios cruise4_head

The trajectories come from the CURRENT kinematic actuator model (720 deg/s, 20 ms lag), i.e. this
answers "can this hardware do what the flight controller is asking for?" -- not yet what happens
when it cannot (that needs the drive model inside the loop, docs/17 section 7).
"""

from __future__ import annotations

import argparse
import copy
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hydrofoil.params import DEFAULT_PARAMS                           # noqa: E402
from hydrofoil.vehicle import VehiclePhysics                         # noqa: E402
from hydrofoil.waves.airy import AiryWave, CalmWater                 # noqa: E402
from hydrofoil.actuation.drive import DriveSpec, budget, limits_at_output   # noqa: E402
from hydrofoil.actuation.loads import (DRIVES, surface_models, drive_angles,   # noqa: E402
                                       steady_moments, output_torque)

SCENARIOS = {
    # name: (target speed, wave or None, duration, bank schedule)
    "takeoff_calm":  (4.0, None, 30.0, None),
    "cruise4_head":  (4.0, dict(direction=0.0), 40.0, None),
    "cruise4_beam":  (4.0, dict(direction=90.0), 40.0, None),
    "bank_8deg":     (4.0, None, 40.0, [(15.0, 8.0), (25.0, -8.0), (35.0, 0.0)]),
    "cruise6_head":  (6.0, dict(direction=0.0), 40.0, None),
}


def fly(name: str, params: dict):
    speed, wave_cfg, t_end, bank = SCENARIOS[name]
    p = copy.deepcopy(params)
    p["target_speed"] = speed
    wave = (AiryWave(amplitude=p["wave_amplitude"], wavelength=p["wave_wavelength"],
                     direction=np.radians(wave_cfg["direction"])) if wave_cfg else CalmWater())
    vp = VehiclePhysics(p)
    hinges = surface_models(p)
    state, t, dt = vp.rest_state(), 0.0, p["dt"]
    rec = {k: [] for k in ("t", "u", "draft")}
    for d in DRIVES:
        rec[d] = []; rec[d + "_M"] = []; rec[d + "_V"] = []
    while t < t_end:
        if bank:
            p["phi_ref"] = 0.0
            for tb, ang in bank:
                if t >= tb:
                    p["phi_ref"] = ang
        state, cmds, thrust, draft, _ = vp.step(state, t, wave)
        t += dt
        env = vp.wave_sample(state[:6], t, wave)
        m = steady_moments(vp, state, cmds, env, hinges)
        ang = drive_angles(cmds)
        rec["t"].append(t); rec["u"].append(state[6]); rec["draft"].append(draft)
        for d in DRIVES:
            rec[d].append(np.radians(ang[d])); rec[d + "_M"].append(m[d]); rec[d + "_V"].append(m[d + "_V"])
    return {k: np.asarray(v) for k, v in rec.items()}, hinges, p


def analyse(rec, hinges, spec: DriveSpec, dt: float, t_skip: float = 0.0):
    sel = rec["t"] >= t_skip
    res = {}
    for d in DRIVES:
        tau, dd, ddd = output_torque(hinges[d], rec[d + "_V"], rec[d], rec[d + "_M"], dt, spec.gear.friction_out)
        b = budget(spec, rec["t"][sel], dd[sel], ddd[sel], tau[sel])
        b["rate_peak_dps"] = float(np.degrees(np.abs(dd[sel]).max()))
        b["accel_peak_dps2"] = float(np.degrees(np.abs(ddd[sel]).max()))
        b["m_hyd_peak"] = float(np.abs(rec[d + "_M"][sel]).max())
        b["m_hyd_mean"] = float(np.mean(rec[d + "_M"][sel]))
        b["delta_range_deg"] = (float(np.degrees(rec[d][sel].min())), float(np.degrees(rec[d][sel].max())))
        res[d] = b
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenarios", nargs="*", default=list(SCENARIOS))
    ap.add_argument("--ratios", nargs="*", type=float, default=[5, 10, 15, 20, 30, 40])
    ap.add_argument("--vbus", type=float, default=24.0)
    a = ap.parse_args()

    base = DriveSpec()
    base.ctrl.v_bus = a.vbus
    dt = DEFAULT_PARAMS["dt"]

    hm = surface_models(DEFAULT_PARAMS)
    print("== Surface models ==")
    for d in ("left", "elev"):
        h = hm[d]
        print(f"  {d:5s} layout={h.g.layout:10s} I_added={h.i_added:.2e} kg m^2  J_struct={h.g.j_struct:.1e}  "
              f"B_hyd/V={h._b_per_V:.2e} N m s/rad per m/s  "
              + (f"Ch_a={h.ch_alpha:.3f} Ch_d={h.ch_delta:.3f} /rad (x{h.g.viscous_factor})" if h.g.layout == "flap"
                 else f"pivot-AC arm={h._arm:+.3f} c"))
        lim = limits_at_output(base, h.g.j_struct + h.i_added)
        print(f"        @N={base.gear.ratio:.0f}: torque max {lim['torque_max']:.1f} N m (cont {lim['torque_cont']:.1f}), "
              f"rate max {np.degrees(lim['rate_max']):.0f} deg/s, J_reflected {lim['j_reflected']:.2e} "
              f"(load {h.g.j_struct + h.i_added:.2e}), inertia-match ratio {lim['n_match']:.1f}")

    runs = {}
    for s in a.scenarios:
        rec, hinges, p = fly(s, DEFAULT_PARAMS)
        runs[s] = (rec, hinges)
        res = analyse(rec, hinges, base, dt)
        print(f"\n== {s}  (N = {base.gear.ratio:.0f}, V_bus = {base.ctrl.v_bus:.0f} V)  "
              f"final u = {rec['u'][-1]:.2f} m/s, foilborne = {rec['draft'][-1] <= 1e-6}")
        print("  drive  delta range [deg]  M_hyd mean/peak [Nm]  tau_out pk/rms [Nm]  rate pk [deg/s]  "
              "I pk/rms [A]  V pk [V]  P_bus pk/mean [W]  regen [J]  ok")
        for d in DRIVES:
            r = res[d]
            print(f"  {d:5s}  {r['delta_range_deg'][0]:6.1f} .. {r['delta_range_deg'][1]:5.1f}   "
                  f"{r['m_hyd_mean']:6.2f} / {r['m_hyd_peak']:5.2f}        {r['tau_out_peak']:6.2f} / {r['tau_out_rms']:5.2f}"
                  f"      {r['rate_peak_dps']:7.0f}       {r['i_peak']:6.1f} / {r['i_rms']:5.1f}  {r['v_peak']:6.1f}"
                  f"   {r['p_bus_peak']:6.0f} / {r['p_bus_mean']:5.1f}    {r['regen_energy_j']:6.2f}   "
                  + ("OK" if r["ok"] else "FAIL " + ",".join(k[3:] for k, v in r.items() if k.startswith("ok_") and not v)))

    print("\n== Gear-ratio sweep (worst drive over all scenarios) ==")
    print("  N     I peak [A]  I rms [A]  V peak [V]  P peak [W]  tau_out pk [Nm]  rpm pk   failing")
    for N in a.ratios:
        spec = DriveSpec()
        spec.ctrl.v_bus = a.vbus
        spec.gear.ratio = N
        worst = {"i_peak": 0, "i_rms": 0, "v_peak": 0, "p_bus_peak": 0, "tau_out_peak": 0, "rpm_peak": 0}
        fails = set()
        for s, (rec, hinges) in runs.items():
            for d, r in analyse(rec, hinges, spec, dt).items():
                for k in worst:
                    worst[k] = max(worst[k], r[k])
                fails |= {k[3:] for k, v in r.items() if k.startswith("ok_") and not v}
        print(f"  {N:4.0f}  {worst['i_peak']:9.1f}  {worst['i_rms']:8.1f}  {worst['v_peak']:9.1f}  "
              f"{worst['p_bus_peak']:9.0f}  {worst['tau_out_peak']:13.2f}  {worst['rpm_peak']:7.0f}   "
              + (",".join(sorted(fails)) or "-"))


if __name__ == "__main__":
    main()
