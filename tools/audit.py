"""
Reproducible audit / verification / tuning harness for the hydrofoil sim.

Everything quoted in docs/13, 14, 15 and 16 was produced with (versions of) these routines.
Run from the repo root:

    python tools/audit.py hydrostatics     # hull stiffness vs metacentric theory      (~1 s)
    python tools/audit.py linearize        # settle, finite-difference A/B, eigenvalues (~30 s)
    python tools/audit.py evaluate         # closed-loop scenario metrics               (~3-5 min)
    python tools/audit.py sweep-angles     # pitch & roll gain grids                    (~3 min)
    python tools/audit.py sweep-heave      # heave gain grid over takeoff               (~5 min)
    python tools/audit.py checks           # quick analytic identities (passivity, RK4, ...) (~10 s)

Nothing here modifies the repo. Gains tried in a sweep are applied to a copy of the params.
"""

from __future__ import annotations

import argparse
import copy
import itertools
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import numpy as np

from hydrofoil.constants import RHO_WATER as RHO, G
from hydrofoil.dynamics.kinematics import R_body_to_ned
from hydrofoil.forces.hull import HullHydrostatics
from hydrofoil.params import DEFAULT_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import AiryWave, CalmWater

DT = DEFAULT_PARAMS["dt"]


# ---------------------------------------------------------------- helpers
def make(gains: dict | None = None, **extra):
    p = dict(DEFAULT_PARAMS)
    p.update(gains or {})
    p.update(extra)
    return p, VehiclePhysics(p)


def advance(vp, s, t, wave, T, rec=None):
    for _ in range(int(round(T / DT))):
        s, c, th, d, _ = vp.step(s, t, wave)
        t += DT
        if rec is not None:
            rec.append((t, s.copy(), c, d))
    return s, t


def settle_time(ts, err, tol, t0=0.0):
    ts, err = np.asarray(ts), np.abs(np.asarray(err))
    bad = np.where(err > tol)[0]
    if len(bad) == 0:
        return 0.0
    return float("inf") if bad[-1] >= len(ts) - 1 else float(ts[bad[-1] + 1] - t0)


def total_variation(x):
    return float(np.sum(np.abs(np.diff(np.asarray(x)))))


def settled(gains=None, T=60.0, **extra):
    p, vp = make(gains, **extra)
    s, t = advance(vp, vp.rest_state(), 0.0, CalmWater(), T)
    return p, vp, s, t


# ---------------------------------------------------------------- hydrostatics (doc 15)
def cmd_hydrostatics(_):
    P = DEFAULT_PARAMS
    L, B, h, dmax, m = P["hull_length"], P["hull_beam"], P["hull_offset"], P["max_draft"], P["mass"]
    A = L * B
    d = m / (RHO * A)
    V = A * d
    KB = h + d / 2                                    # centre of buoyancy above the CoG
    th = dict(Kz=RHO * G * A, Kphi=RHO * G * V * (KB + L * B**3 / 12 / V), Kth=RHO * G * V * (KB + B * L**3 / 12 / V))
    H = HullHydrostatics(L, B, P["hull_x"], h, dmax)
    eta0 = np.array([0, 0, h + d, 0, 0, 0.0])
    e = 1e-4

    def K(idx, k):
        v = np.zeros(6); v[idx] = e
        return -(H.buoyancy(eta0 + v, 0.0)[k] - H.buoyancy(eta0 - v, 0.0)[k]) / (2 * e)

    print(f"rest draft {d:.4f} m, volume {V:.5f} m^3, GM_T {KB + L*B**3/12/V:.3f} m, GM_L {KB + B*L**3/12/V:.3f} m")
    print(f"theory  K_z {th['Kz']:9.1f}  K_roll {th['Kphi']:8.1f}  K_pitch {th['Kth']:9.1f}")
    print(f"model   K_z {K(2,2):9.1f}  K_roll {K(3,3):8.1f}  K_pitch {K(4,4):9.1f}   ({H.n} cells)")
    print("net vertical force at rest draft (should be 0):", H.buoyancy(eta0, 0.0)[2] + m * G)
    for dd in (0.01, 0.05, 0.15, 0.3):
        eta = np.array([0, 0, h + dd, 0, 0, 0.0])
        print(f"  draft {dd:.2f}: F {-H.buoyancy(eta, 0.0)[2]:8.2f}  vs rho g A d {RHO*G*A*dd:8.2f}")


# ---------------------------------------------------------------- linearisation (doc 13)
def cmd_linearize(_):
    p, vp, s0, t = settled(T=90.0)
    cmds = {k: vp.controller.__dict__[a]._position for k, a in
            (("delta_main", "main_actuator"), ("delta_elev", "elev_actuator"), ("delta_roll", "roll_actuator"))}
    thrust = vp.surge.pid.update(p["target_speed"], s0[6]) if False else 0.0
    # use the converged commands from the last step
    _, c, th, _, _ = vp.step(s0, t, CalmWater())
    f0 = vp.physics_rhs(s0, c, th)
    print(f"settled: z {s0[2]:.3f} u {s0[6]:.3f} theta {np.degrees(s0[4]):.3f} deg ; |xdot| (x,psi aside) {np.round(f0[1:], 4)}")
    n = 12
    A = np.zeros((n, n)); B = np.zeros((n, 4))
    base_u = np.array([c["delta_main"], c["delta_elev"], c["delta_roll"], th])

    def f(x, u):
        return vp.physics_rhs(x, {"delta_main": u[0], "delta_elev": u[1], "delta_roll": u[2]}, u[3])

    for i in range(n):
        e = np.zeros(n); e[i] = 1e-6
        A[:, i] = (f(s0 + e, base_u) - f(s0 - e, base_u)) / 2e-6
    for j in range(4):
        e = np.zeros(4); e[j] = 1e-5
        B[:, j] = (f(s0, base_u + e) - f(s0, base_u - e)) / 2e-5
    ev = np.linalg.eigvals(A)
    print("open-loop plant eigenvalues (no controller / actuators):")
    for l in sorted(ev, key=lambda z: z.real):
        print("   ", np.round(l, 3))
    idx = [2, 3, 4, 7, 8, 9, 10, 11]
    Ar, Br = A[np.ix_(idx, idx)], B[np.ix_(idx, [0, 1, 2])]
    C = np.hstack([np.linalg.matrix_power(Ar, k) @ Br for k in range(len(idx))])
    print("controllability rank (z,phi,theta,v,w,p,q,r ; 3 flaps):", np.linalg.matrix_rank(C, tol=1e-8), "of", len(idx))
    print("cond(M):", np.linalg.cond(vp.rigid_body.M))


# ---------------------------------------------------------------- quick identities (doc 13 sec 1)
def cmd_checks(_):
    p, vp = make()
    rb = vp.rigid_body
    nu = np.array([4, .3, .2, .1, -.2, .3])
    print("nu^T C nu (must be 0):", nu @ rb.C(nu) @ nu)
    nu2 = np.array([4.0, 0, 0.5, 0, 0, 0])
    print("Munk: (C nu)_q =", (rb.C(nu2) @ nu2)[4], " expected -m_a u w =", -rb.added_mass_heave * 4 * 0.5)
    print("added mass heave/pitch:", rb.added_mass_heave, rb.added_inertia_pitch)
    s = vp.rest_state(); s[3] = 0.2; s[4] = 0.1; s[6] = 3; s[8] = 0.3
    cm = {"delta_main": 10.0, "delta_elev": 2.0, "delta_roll": 1.0}

    def prop(dt, T=2.0):
        x = s.copy()
        for _ in range(int(round(T / dt))):
            fn = lambda y: vp.physics_rhs(y, cm, 20.0)
            k1 = fn(x); k2 = fn(x + dt / 2 * k1); k3 = fn(x + dt / 2 * k2); k4 = fn(x + dt * k3)
            x = x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return x
    print("RK4 dt 0.01 vs 0.0005 max state error after 2 s:", np.abs(prop(0.01) - prop(0.0005)).max())
    w = AiryWave(wavelength=5.0)
    print(f"wave dispersion: omega^2/(g k) = {w.omega**2/(G*w.k):.6f} (1 = deep-water)")


# ---------------------------------------------------------------- closed-loop scenarios (doc 14 sec H)
def evaluate(gains=None, verbose=True, waves=True, T_takeoff=60.0):
    r = {}
    p, vp = make(gains)
    rec = []
    s, t = advance(vp, vp.rest_state(), 0.0, CalmWater(), T_takeoff, rec)
    ts = [x[0] for x in rec]; z = np.array([x[1][2] for x in rec]); zref = p["hull_offset"] - p["flying_height"]
    ft = next((x[0] for x in rec if x[3] <= 1e-6), np.inf)
    r["foilborne_t"] = ft
    r["z_settle_2cm"] = settle_time(ts, z - zref, 0.02)
    r["z_final_err"] = float(z[-1] - zref)
    r["touch_after_foilborne"] = float(max([x[3] for x in rec if x[0] > ft + 3] + [0.0])) if np.isfinite(ft) else float("nan")

    def fresh():
        return settled(gains, T=T_takeoff)

    p, vp, s, t = fresh(); th0 = np.degrees(s[4]); s[3] = np.radians(5.0); rec = []
    advance(vp, s, t, CalmWater(), 15.0, rec)
    ph = [np.degrees(x[1][3]) for x in rec]; tt = [x[0] for x in rec]
    r["roll5_settle_0p5deg"] = settle_time(tt, ph, 0.5, tt[0])

    p, vp, s, t = fresh(); z0 = s[2]; p["phi_ref"] = 8.0; rec = []
    advance(vp, s, t, CalmWater(), 20.0, rec)
    ph = np.array([np.degrees(x[1][3]) for x in rec]); tt = np.array([x[0] for x in rec])
    r["bank8_final"] = float(ph[-1]); r["bank8_settle_1deg"] = settle_time(tt, ph - 8.0, 1.0, tt[0])
    r["bank8_height_loss"] = float(max(x[1][2] for x in rec) - z0); r["bank8_touch"] = float(max(x[3] for x in rec))
    r["bank8_heading_change_deg"] = float(np.degrees(rec[-1][1][5]))

    p, vp, s, t = fresh(); s[4] += np.radians(3.0); rec = []
    advance(vp, s, t, CalmWater(), 10.0, rec)
    thv = np.array([np.degrees(x[1][4]) for x in rec]) - th0
    r["pitch3_settle_0p5deg"] = settle_time([x[0] for x in rec], thv, 0.5, rec[0][0])

    p, vp, s, t = fresh(); p["flying_height"] = 0.35; zr = p["hull_offset"] - 0.35; rec = []
    advance(vp, s, t, CalmWater(), 25.0, rec)
    r["hstep_settle_2cm"] = settle_time([x[0] for x in rec], np.array([x[1][2] for x in rec]) - zr, 0.02, rec[0][0])

    if waves:
        # direction = the way the wave TRAVELS: 0 = same way as the boat (following sea, slow encounter, the worst
        # case for height), 180 = against it (head sea, fast encounter). ("head" used to be direction 0.)
        zr = p["hull_offset"] - p["flying_height"]
        for name, ddeg in (("following", 0.0), ("head", 180.0), ("beam", 90.0), ("oblique", 45.0)):
            p, vp, s, t = fresh(); rec = []
            advance(vp, s, t, AiryWave(0.05, 5.0, direction=np.radians(ddeg)), 40.0, rec)
            r[f"wave_{name}_z_rms_mm"] = float(1e3 * np.std([x[1][2] - zr for x in rec[500:]]))
            r[f"wave_{name}_max_phi_deg"] = float(max(abs(np.degrees(x[1][3])) for x in rec))
            r[f"wave_{name}_touch"] = float(max(x[3] for x in rec))
    if verbose:
        for k, v in r.items():
            print(f"  {k:28s} {v}")
    return r


def cmd_evaluate(a):
    print("gains:", {k: v for k, v in DEFAULT_PARAMS.items() if k.startswith(("kp_", "ki_", "kd_"))})
    evaluate(waves=not a.no_waves)
    # The default actuator model is "servo", which ignores actuator_params; the doubled delay/lag only
    # means something on the kinematic actuator, so run the nominal AND the doubled case on it.
    DEFAULT_PARAMS["actuator_model"] = "kinematic"
    print("-- kinematic actuator, nominal (20 ms delay, 20 ms lag)")
    evaluate(waves=False)
    print("-- kinematic actuator, delay and lag doubled")
    DEFAULT_PARAMS["actuator_params"] = {"rate_limit_deg_s": 720.0, "tau": 0.04, "delay_s": 0.04}
    evaluate(waves=False)


# ---------------------------------------------------------------- sweeps (doc 14 E2)
def with_gains(snap, **g):
    vp, s, t = copy.deepcopy(snap)
    c = vp.controller
    for loop, pid in (("heave", c.heave_pid), ("pitch", c.pitch_pid), ("roll", c.roll_pid)):
        for k in ("Kp", "Ki", "Kd"):
            key = f"{k.lower()}_{loop}"
            if key in g:
                setattr(pid, k, g[key])
    return vp, s, t


def cmd_sweep_angles(_):
    # IMPORTANT: settle with gentle, stable gains first. Settling with unstable gains stores a limit cycle in the
    # controller/actuator state and contaminates every trial (this happened once; see docs/14 E2).
    _, vp, s, t = settled({"kp_roll": 1.0, "kd_roll": 0.1, "kp_pitch": 0.5, "kd_pitch": 0.05}, T=60.0)
    snap = (vp, s, t)
    print("pitch: kp kd -> (ISE, residual, flap TV, flap peak)")
    for kp, kd in itertools.product((0.3, 0.6, 1.0, 1.5, 2.0), (0.0, 0.1, 0.3, 0.6)):
        v, x, tt = with_gains(snap, kp_pitch=kp, kd_pitch=kd)
        th0 = np.degrees(x[4]); x[4] += np.radians(3.0); rec = []
        advance(v, x, tt, CalmWater(), 10.0, rec)
        th = np.array([np.degrees(r[1][4]) for r in rec]) - th0; de = [r[2]["delta_elev"] for r in rec]
        print(f"  kp {kp:4} kd {kd:4}", round(float(np.sum(th**2) * DT), 3), round(float(np.max(np.abs(th[-300:]))), 3),
              round(total_variation(de), 2), round(float(np.max(np.abs(de))), 2))
    print("roll: kp kd -> (ISE, residual, flap TV, flap peak)")
    for kp, kd in itertools.product((1.0, 2.0, 3.5, 5.0), (0.1, 0.2, 0.5, 1.0)):
        v, x, tt = with_gains(snap, kp_roll=kp, kd_roll=kd)
        x[3] = np.radians(5.0); rec = []
        advance(v, x, tt, CalmWater(), 8.0, rec)
        ph = np.array([np.degrees(r[1][3]) for r in rec]); dr = [r[2]["delta_roll"] for r in rec]
        print(f"  kp {kp:4} kd {kd:4}", round(float(np.sum(ph**2) * DT), 3), round(float(np.max(np.abs(ph[-300:]))), 3),
              round(total_variation(dr), 2), round(float(np.max(np.abs(dr))), 2))


def cmd_sweep_heave(_):
    base = {"kp_roll": 2.0, "kd_roll": 0.2, "kp_pitch": 1.0, "kd_pitch": 0.1}
    print("heave: kp ki kd -> foilborne time, 2cm settle, overshoot-above-target, final err, flap peak")
    for kp, ki, kd in itertools.product((15, 40, 80), (1, 6, 15), (5, 20)):
        p, vp = make({**base, "kp_heave": kp, "ki_heave": ki, "kd_heave": kd})
        rec = []
        advance(vp, vp.rest_state(), 0.0, CalmWater(), 50.0, rec)
        ts = np.array([x[0] for x in rec]); z = np.array([x[1][2] for x in rec]); zref = p["hull_offset"] - p["flying_height"]
        ft = next((x[0] for x in rec if x[3] <= 1e-6), float("inf"))
        dm = np.array([x[2]["delta_main"] for x in rec])
        print(f"  kp {kp:3} ki {ki:3} kd {kd:3}  ft {ft:5.1f}  settle {settle_time(ts, z - zref, 0.02):6.1f}  above {max(0, zref - z.min()):.3f}  "
              f"final {z[-1] - zref:+.4f}  dmax {dm.max():.1f}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sp = ap.add_subparsers(dest="cmd", required=True)
    for name, fn in (("hydrostatics", cmd_hydrostatics), ("linearize", cmd_linearize), ("checks", cmd_checks),
                     ("evaluate", cmd_evaluate), ("sweep-angles", cmd_sweep_angles), ("sweep-heave", cmd_sweep_heave)):
        q = sp.add_parser(name)
        if name == "evaluate":
            q.add_argument("--no-waves", action="store_true")
        q.set_defaults(fn=fn)
    a = ap.parse_args()
    a.fn(a)


if __name__ == "__main__":
    main()
