"""
Re-check every audit claim against the ORIGINAL code, not the fixed code (docs/18).

    python tools/verify_original_audit.py               # extracts commit 841d501 to a temp dir and runs the checks there
    python tools/verify_original_audit.py --rev <sha>   # another snapshot

841d501 is upstream's code plus docs only, so its hydrofoil/ package is the code as found. The checks run in a
separate interpreter (-I) with only that snapshot on sys.path; the current repo's pid.py is loaded by file path for
the one side-by-side comparison (anti-windup). Takes ~3-5 min (several closed-loop runs).
"""

from __future__ import annotations

import argparse
import copy
import importlib.util
import os
import shutil
import subprocess
import sys
import tarfile
import tempfile
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]


def extract(rev: str) -> Path:
    d = Path(tempfile.mkdtemp(prefix="hydrofoil_orig_"))
    tar = subprocess.run(["git", "-C", str(REPO), "archive", rev], check=True, capture_output=True).stdout
    tf = d / "snap.tar"
    tf.write_bytes(tar)
    with tarfile.open(tf) as t:
        t.extractall(d / "src", filter="data")
    return d / "src"


def checks(orig: Path):
    sys.path.insert(0, str(orig))
    import numpy as np
    import hydrofoil
    assert Path(hydrofoil.__file__).resolve().is_relative_to(orig.resolve()), hydrofoil.__file__
    from hydrofoil.dashboard import DEFAULT_PARAMS
    from hydrofoil.vehicle import VehiclePhysics
    from hydrofoil.waves.airy import AiryWave, CalmWater
    from hydrofoil.control.pid import PID
    from hydrofoil.control.surge import SurgeController
    from hydrofoil.control.actuator import Actuator
    from hydrofoil.forces.hull import hull_draft, hull_buoyancy_force
    from hydrofoil.forces.buoyancy import gravity_force
    from hydrofoil.forces.foil import FoilTables

    P = copy.deepcopy(DEFAULT_PARAMS)
    print("original package:", hydrofoil.__file__)

    def fly(T, wave=None, state=None, t=0.0, vp=None, rec=None):
        vp = vp or VehiclePhysics(P)
        s = vp.rest_state() if state is None else state
        wave = wave or CalmWater()
        while t < T - 1e-9:
            s, c, th, dr, we = vp.step(s, t, wave); t += P["dt"]
            if rec is not None: rec.append((t, s.copy(), c, dr))
        return vp, s, t

    rec = []
    vp, s, t = fly(120.0, rec=rec)
    zref = P["hull_offset"] - P["flying_height"]
    ts = np.array([x[0] for x in rec]); err = np.array([x[1][2] - zref for x in rec])
    ft = next(x[0] for x in rec if x[3] <= 1e-6)
    out = np.where(np.abs(err) > 0.02)[0]
    print(f"[E2] foilborne {ft:.2f} s; height within 2 cm after {ts[out[-1]]:.1f} s; z err at 15 s {err[np.searchsorted(ts, 15)]:.3f} m; "
          f"trim flap {rec[-1][2]['delta_main']:.1f} deg")
    cruise = (vp, s, t)

    for label, scale in (("original gains", 1.0), ("gains x57.3 (= deg/deg)", 57.2958)):
        v2, s2, t2 = copy.deepcopy(cruise)
        for pid in (v2.controller.roll_pid, v2.controller.pitch_pid):
            pid.Kp *= scale; pid.Kd *= scale
        s2 = s2.copy(); s2[3] = np.radians(5.0); r = []
        fly(t2 + 15.0, state=s2, t=t2, vp=v2, rec=r)
        ph = np.degrees([x[1][3] for x in r])
        print(f"[E1] roll 5 deg kick, {label}: phi after 5 / 10 / 15 s = {ph[499]:.1f} / {ph[999]:.1f} / {ph[-1]:.1f} deg")
    v2, s2, t2 = copy.deepcopy(cruise); th0 = s2[4]; s2 = s2.copy(); s2[4] += np.radians(3.0); r = []
    fly(t2 + 15.0, state=s2, t=t2, vp=v2, rec=r)
    th = np.degrees([x[1][4] - th0 for x in r])
    print(f"[E1] pitch 3 deg kick, original gains: dtheta after 1 / 5 / 15 s = {th[99]:.2f} / {th[499]:.2f} / {th[-1]:.2f} deg")

    spec = importlib.util.spec_from_file_location("pid_fixed", REPO / "hydrofoil" / "control" / "pid.py")
    fixed = importlib.util.module_from_spec(spec); spec.loader.exec_module(fixed)
    for name, cls in (("original", PID), ("fixed", fixed.PID)):
        p = cls(Kp=1.0, Ki=1.0, Kd=1.0, dt=0.01, output_min=-1.0, output_max=1.0); m = 0.0; p.update(0.1, m)
        for _ in range(2000):        # error +0.1 while D (plant moving the right way) keeps the output in range: I grows to 1.4
            m -= 0.005; p.update(0.1, m)
        o = np.array([p.update(-0.2, m) for _ in range(3000)])   # plant stops, error reverses
        leave = f"{np.argmax(o < 1.0) * 0.01:.2f} s" if (o < 1.0).any() else "never (30 s)"
        print(f"[E3] {name} PID: integral {p._integral:.2f}, saturated with error -0.2, leaves saturation after {leave}")
    p = PID(Kp=1.0, Ki=1.0, Kd=0.0, dt=0.01, output_min=-1.0, output_max=1.0)
    for _ in range(300): p.update(5.0, 0.0)
    print(f"[E3] original, plain saturation: integral stays {p._integral:.2f}; output right after error -> -0.2: {p.update(-0.2, 0.0):.2f}")

    sc = SurgeController(Kp=0.0, Ki=0.0, Kd=1.0, dt=0.01, max_thrust=50); sc.update(4.0, 1.0)
    print(f"[E4] surge Kd=1, speed rising 1 m/s^2: D-term thrust {sc.update(4.0, 1.01):+.2f} N (should be negative)")

    for dt in (0.01, 0.002):
        a = Actuator(dt=dt, rate_limit_deg_s=1e9, tau=0.0, delay_steps=2); n = 0
        while a.update(1.0) < 0.5: n += 1
        print(f"[E5] dt {dt}: default latency {n * dt * 1000:.0f} ms")

    rb = vp.rigid_body
    nu = np.array([4.0, 0.3, 0.2, 0.1, -0.2, 0.3])
    nu2 = np.array([4.0, 0, 0.1, 0, 0, 0])
    print(f"[E7] nu^T C nu = {nu @ rb.C(nu) @ nu:.1e} (energy conserved); pitch row of C nu at u 4, w 0.1: {(rb.C(nu2) @ nu2)[4]:.2f} N m, "
          f"Munk -(Z_wdot - X_udot) u w = {-rb.added_mass_heave * 0.4:.2f} N m")

    a2d = FoilTables.naca0009().cl_slope_per_deg * 180 / np.pi
    for ar in (4.0, 3.0):
        print(f"[E8] AR {ar}: lift factor AR/(AR+2) = {ar / (ar + 2):.3f} vs Helmbold(e 0.8, polar slope {a2d:.2f}) = {1 / (1 + a2d / (np.pi * 0.8 * ar)):.3f}")

    f = vp.right_foil
    na = np.array([4.0, 0, 0.1, 0, 0, 0]); nb = na.copy(); nb[1] = 1.0
    za, zb = f.generalized_force(na, delta=10.0)[2], f.generalized_force(nb, delta=10.0)[2]
    print(f"[E9] foil Z force v=0: {za:.1f} N, side-slip v=1: {zb:.1f} N ({(zb / za - 1) * 100:+.1f} %)")

    sc_ = cruise[1].copy(); c0 = {"delta_main": 14.0, "delta_elev": 0.0, "delta_roll": 0.0}; sv = sc_.copy(); sv[7] = 1.0
    f0 = vp.physics_rhs(sc_, c0, 17.0, 0.0, np.zeros(3)); f1 = vp.physics_rhs(sv, c0, 17.0, 0.0, np.zeros(3))
    print(f"[E10] side force from 1 m/s side-slip: {(f1[7] - f0[7]) * P['mass']:.2f} N "
          f"(one 0.12 m chord strut, 0.5 m wetted, 4 m/s: ~{0.5 * 1025 * 16 * 0.12 * 0.5 * 0.6 * np.arctan(0.25):.0f} N)")

    eta = np.array([0, 0, P["hull_offset"] + 0.04, np.radians(15), 0, 0])
    zb_ = eta[2] - P["hull_offset"] * np.cos(eta[3])
    print(f"[E11] 15 deg roll: original draft {hull_draft(eta, P['hull_offset'], P['max_draft']):.4f} m vs rotated hull bottom {zb_:.4f} m")
    for name, idx, k in (("roll", 3, 197.4), ("pitch", 4, 2163.0)):
        e = np.array([0, 0, P["hull_offset"] + 0.0407, 0, 0, 0]); e[idx] = np.radians(5)
        tau = hull_buoyancy_force(e, P["hull_offset"], P["waterplane_area"], P["max_draft"]) + gravity_force(P["mass"], e)
        print(f"[E12] floating, {name} 5 deg: hydrostatic moment {tau[idx]:.3f} N m (metacentric theory {k * np.radians(5):.0f} N m restoring)")

    w = AiryWave(amplitude=0.05, wavelength=5.0, period=2.0)
    print(f"[E13] orbital velocity applied at CoG depth; at the main foil (0.5 m deeper) it is x{np.exp(-w.k * 0.5):.2f}; "
          f"elevator 1.2 m aft is {np.degrees(w.k * 1.2):.0f} deg out of phase")

    for ki, ddir in ((0.0, 0.0), (0.05, 0.0), (0.05, 90.0)):
        v3, s3, t3 = copy.deepcopy(cruise); v3.controller.roll_pid.Ki = ki; r = []
        fly(t3 + 90.0, wave=AiryWave(amplitude=0.05, wavelength=5.0, period=2.0, direction=np.radians(ddir)),
            state=s3, t=t3, vp=v3, rec=r)
        print(f"[E14] waves dir {ddir:.0f} deg, roll Ki {ki}: max |phi| over 90 s = {max(abs(np.degrees(x[1][3])) for x in r):.3f} deg")

    print(f"[E15] lambda 5 m, T 2 s: phase speed 2.50 m/s vs deep-water {np.sqrt(9.81 * 5 / (2 * np.pi)):.2f} m/s")

    v4 = VehiclePhysics(P)
    for _ in range(50):
        c = v4.controller.update(-10.0, 0.0, 0.0, 1.0, 0.0, 0.0)
    print(f"[W7] heave + roll both demanding: delta_main {c['delta_main']:.1f}, delta_roll {c['delta_roll']:.1f} -> physical right flap "
          f"{c['delta_main'] + c['delta_roll']:.1f} deg (channels clamped separately)")
    print(f"[D13] hull drag at rest draft, 4 m/s: {P['hull_drag_coeff'] * (0.0407 / P['max_draft']) * 16:.1f} N")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rev", default="841d501")
    ap.add_argument("--orig", help=argparse.SUPPRESS)
    a = ap.parse_args()
    if a.orig:
        checks(Path(a.orig))
        return
    src = extract(a.rev)
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    try:
        rc = subprocess.call([sys.executable, "-I", __file__, "--orig", str(src)], env=env)
    finally:
        shutil.rmtree(src.parent, ignore_errors=True)
    sys.exit(rc)


if __name__ == "__main__":
    main()
