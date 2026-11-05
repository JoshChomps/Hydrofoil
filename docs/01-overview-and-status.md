# 01 — Overview and Status

## What this is

A Python 6-DOF physics simulation (Fossen notation) of a **fully submerged
T-foil hydrofoil boat**: a hull riding above the water on struts, a main foil
split into left/right halves (differential flap = roll authority), and an aft
elevator (pitch authority). It is closed-loop with three independent PID loops (not a true cascade), has a
live browser dashboard, and a native MuJoCo 3D viewer.
Built for the UBC Okanagan Marine Robotics Hydrofoil Club (per README).

Python is the sole physics authority. The MuJoCo viewer is a renderer plus a keyboard pilot input;
it never simulates anything (Unity and the UDP bridge were removed on branch `mujoco-viewer`).

## Three ways to run it

| Entry point | Purpose | Uses `vehicle.py`? |
|---|---|---|
| `python -m hydrofoil.dashboard` | FastAPI + WebSocket app on `localhost:8000`, sliders, live plots, keyboard "stick". The actively developed entry point. | yes |
| `python -m hydrofoil.viewer` | MuJoCo 3D viewer, in-process, keyboard pilot (replaced the Unity/UDP path). | yes |
| `python -m hydrofoil.sim` | Headless batch run (20 s), saves PNG + Plotly HTML into `data/`. Older, simpler model. | **no** (own model) |

## What exists (inventory)

- **Rigid body** dynamics: `M ν̇ + C(ν) ν = τ`, with diagonal added mass in heave and pitch only.
- **Kinematics**: ZYX Euler Jacobian.
- **Foil model**: single-strip lift/drag from an XFLR5 NACA 0009 polar, with flap
  effectiveness, finite-span (AR/(AR+2)), free-surface loss and induced-drag corrections.
- **Hull model**: draft-dependent linear (wall-sided) buoyancy and quadratic drag; enables the
  displacement → foilborne "takeoff" transition.
- **Waves**: single regular Airy wave (elevation + orbital velocity), or calm.
- **Control**: three SISO PIDs (heave → main flap, pitch → elevator, roll → differential flap),
  one speed PID (→ thrust), per-channel actuator model (delay + rate limit + lag).
- **Safety net** in `VehiclePhysics.step()`: NaN rollback + attitude/velocity clamps.
- **Dashboard**: params panel (live + config tabs), 3 canvas views, 4 Plotly charts, telemetry strip, keyboard control.
- **UDP bridge**: `stream.py` (state out, ~30 Hz) and `control_uplink.py` (pilot in).
- **MuJoCo viewer** (`hydrofoil/viewer.py`): render-only, in-process, keyboard pilot (doc 08). Replaced the former Unity project.
- **Tests**: `tests/test_forces.py`, `tests/test_actuator.py` (pytest). 
- **Data**: one polar, `data/foil_tables/naca0009_xflr5_re2700000.dat`.

## What is stub / missing

- `hydrofoil/estimation/` and `hydrofoil/sensors/` — **empty packages**. Controllers get exact ground truth.
- No linearised plant (A, B matrices) — prerequisite for LQR/MPC; not started.
- No LQR/MPC/MIMO controller.
- No measured vehicle parameters (mass, inertia, geometry, hull, actuator are all guesses).
- No hardware/firmware code (target per upstream notes: NUCLEO-H723ZG + two Maytech VESCs over CAN; deprioritised).
- No wave spectrum, current, or yaw/turn-specific propulsion (single scalar thrust along +x).
- No post-stall model; no Reynolds correction; one polar used for all surfaces.
- `notebooks/` directory is mentioned in the README but does not exist in the repo.

## Key findings of the 2026-11-04 audit (details: doc 13; **fixes: doc 14**)

> Status after the fix pass: items 1, 2, 3, 5 (Munk), 6 (struts, per-foil waves) and 7 are **fixed**; item 4 (trim feed-forward) is improved by the retune (settling 62 s → ~17 s) but still integrator-driven. The table below records the findings as originally made.

| # | Finding | Severity |
|---|---|---|
| 1 | Roll and pitch PIDs take error in **rad** and output **deg** → effective gains ~57× smaller than the numbers imply | High |
| 2 | Roll loop does **not** stabilise at current gains (5° disturbance grows to 14° in 15 s; 4° bank command ends with the hull back in the water). Upstream "stable to ~10° bank" claims do not reproduce | High |
| 3 | Waves are symmetric about y = 0 and cannot excite roll/yaw, so wave-based roll tests are vacuous | High |
| 4 | No feed-forward: heave trim flap (~14° of ±20°) is built by the integrator; height settles in 60–90 s (default run is 60 s) | Medium |
| 5 | No Munk moment (added-mass Coriolis); shifts the unstable pole 0.178 → 0.209 s⁻¹ and the height/speed mode 0.98 → 0.73 rad/s | Medium |
| 6 | No strut side force, hull restoring moment, wing–tail downwash; single-point wave sampling | Medium |
| 7 | The "cascade PID" is three parallel SISO PIDs: PI-D (D on measurement), conditional-undo anti-windup, no filter, no scheduling | Info |
| 8 | Verified correct: Coriolis passivity, kinematics, force directions, RK4 accuracy (~1e-5), flap authority (min foilborne speed ≈ 3.1 m/s), 23/23 tests pass | Info |

## Progress summary (from the code and git history)

| Date | Milestone |
|---|---|
| 2026-09-17 | `PROJECT_STATE.md` snapshot written (before added-mass / flap-effectiveness / free-surface / induced-drag / tests landed — see doc 11) |
| 2026-09-10 | Unity screenshots captured (file names) |
| 2026-10-24 | Initial public commit (`3170acd`) |
| 2026-10-25 | Fix `.gitignore` that excluded the foil polar (`cc314c2`) |
| 2026-11-04 | README update (`2af5157`); documentation pass; math audit; Unity replaced by MuJoCo viewer (branch `mujoco-viewer`) |

Roughly in build order, inferred from comments: open-loop/PID heave+pitch → roll unlock with differential
main foil → hull buoyancy/takeoff → surge PID → waves coupled through relative flow → UDP bridge + Unity (since replaced by the MuJoCo viewer)
driver → pilot uplink (height, speed, bank) → safety net → foil corrections (flap effectiveness, finite
span, free surface, induced drag) → added mass → actuator model → unit tests.

## Current headline limitations

1. Gains were **retuned on 2026-11-04** after fixing a rad/deg unit mismatch (doc 14); the earlier values were for a different plant.
2. Roll Ki is still 0 (no longer needed). The upstream bank/fuzz claims were not reproduced; the new tests cover disturbance rejection, an 8° bank and oblique waves, but randomised combined-input fuzzing has not been re-run.
3. Everything physical is a placeholder number.
