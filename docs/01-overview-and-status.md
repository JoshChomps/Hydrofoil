# 01 — Overview and Status

## What this is

A Python 6-DOF physics simulation (Fossen notation) of a **fully submerged
T-foil hydrofoil boat**: a hull riding above the water on struts, a main foil
split into left/right halves (differential flap = roll authority), and an aft
elevator (pitch authority). It is closed-loop with cascade PID control, has a
live browser dashboard, and can stream state over UDP to a Unity visualizer.
Built for the UBC Okanagan Marine Robotics Hydrofoil Club (per README).

Python is the sole physics authority. Unity is a renderer plus a pilot-input
source; it never simulates anything in the normal mode.

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
- **Unity project** (Unity 6000.3.6f1): `PythonSimDriver` + `SimModeController` are the live path;
  several legacy arcade-physics / Simulink scripts remain.
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

## Progress summary (from the code and git history)

| Date | Milestone |
|---|---|
| 2026-09-17 | `PROJECT_STATE.md` snapshot written (before added-mass / flap-effectiveness / free-surface / induced-drag / tests landed — see doc 11) |
| 2026-09-10 | Unity screenshots captured (file names) |
| 2026-10-24 | Initial public commit (`3170acd`) |
| 2026-10-25 | Fix `.gitignore` that excluded the foil polar (`cc314c2`) |
| 2026-11-04 | README update (`2af5157`); this documentation pass |

Roughly in build order, inferred from comments: open-loop/PID heave+pitch → roll unlock with differential
main foil → hull buoyancy/takeoff → surge PID → waves coupled through relative flow → UDP bridge + Unity
driver → pilot uplink (height, speed, bank) → safety net → foil corrections (flap effectiveness, finite
span, free surface, induced drag) → added mass → actuator model → unit tests.

## Current headline limitations

1. Gains **have not been retuned** since the foil-force corrections and added mass were added. Doc 13 shows the roll loop does not stabilise at all today (and a rad/deg unit mismatch in the roll and pitch PIDs).
2. Roll Ki must be 0; the upstream bank limits are not reproducible on the current code; combined rapid input can still diverge (masked by clamps).
3. Everything physical is a placeholder number.
