# 12 — Progress Log

Append new entries at the bottom. Keep them dated.

## Timeline reconstructed from the repository

| Date | Source | Event |
|---|---|---|
| 2026-09-10 | Unity screenshot file names | Visual-checks of the Unity scene (6 screenshots, 15:25–15:45) |
| 2026-09-17 | `hydrofoil/PROJECT_STATE.md` | Snapshot written to orient LQR/MPC work: PID done, roll unlocked, safety net, UDP bridge |
| 2026-10-24 | commit `3170acd` (jleathem) | Initial commit: Python sim + Unity visualiser |
| 2026-10-25 | commit `cc314c2` (jleathem) | `.gitignore` fix so the foil polar data file is tracked |
| 2026-11-04 | commit `2af5157` (jacobleathem) | README update ("missing file" note for the 709 MB hull model) |

Capabilities present in the code at this commit that post-date the 2026-09-17 snapshot: diagonal added mass,
flap effectiveness (0.21/1.0), finite-span AR/(AR+2), free-surface lift loss, induced drag with Oswald e, actuator model
(delay + rate limit + lag), pilot bank/speed uplink, and the unit tests.

## Milestone status

| Milestone | Status |
|---|---|
| 6-DOF rigid body + kinematics | done |
| Foil model with corrections | done (placeholders for free-surface; single polar) |
| Hull buoyancy / takeoff | done (linear wall-sided) |
| Waves | regular Airy only |
| PID (heave/pitch/roll, three parallel loops) + surge | done, **not retuned**; roll loop unstable and rad/deg unit bug (doc 13) |
| Actuator model | done, placeholder numbers |
| Dashboard | done |
| Unity bridge + pilot uplink | done, then **removed** 2026-11-04; replaced by MuJoCo viewer (branch `mujoco-viewer`, not yet merged/pushed) |
| Safety net | done |
| Unit tests | partial (forces + actuator only) |
| Linearisation (A, B) | not started |
| LQR / MPC | not started |
| Sensors / estimator | empty packages |
| Hardware / firmware | not started (deprioritised) |
| Measured vehicle parameters | none |

## Log

### 2026-11-04 — Documentation pass
- Cloned `https://github.com/jacobleathem/hydrofoil-sim` (git was not initialised in `D:\Coding_Projects\Hydrofoil Sim`; the folder was empty so it was cloned in place, equivalent to the requested "pull").
- Read every Python module, both test files, the polar, and all Unity C# scripts.
- Created `docs/` (this folder), 12 documents.
- Attempted to run tests/headless probe; blocked by the permission check, so no runtime verification was done.
- Findings recorded in doc 11 (stale `PROJECT_STATE.md`, missing `plotly` in requirements, surge-D sign, etc.).
- No source files were modified.

### 2026-11-04 — Math audit, MuJoCo viewer
- Ran the code (it had been blocked earlier): 23/23 tests pass; numerical audit in doc 13.
- Findings: rad/deg gain mismatch in roll/pitch PIDs; roll loop unstable at current gains; waves cannot excite roll; no feed-forward (heave settles in 60-90 s); Munk moment missing.
- New branch `mujoco-viewer`: removed `unity-visualizer/`, `stream.py`, `control_uplink.py`, `sim_realtime.py`; added `hydrofoil/viewer.py` (MuJoCo, render-only) and `hydrofoil/params.py`; requirements now list `mujoco`, `plotly`, `pytest`.
- Physics files were not changed. Docs updated across 01, 04, 05, 06, 09, 10, 11, 12 with the findings; new doc 13.
- Follow-ups: fix PID units and retune; add committed regression tests for bank/disturbance; merge or push `mujoco-viewer`.

### 2026-11-04 — Method/unit/math fix pass and retune
- Fixed (doc 14): PID rad/deg mismatch; anti-windup; surge Kd sign; actuator delay in seconds; Munk (added-mass Coriolis); consistent finite-span/Oswald; spanwise flow in foil force; hull draft vs attitude and buoyancy moment arm; per-foil wave sampling; wave direction + deep-water dispersion; stale reporting; `sim.py` rebuilt on `VehiclePhysics`; shared constants.
- Added: vertical struts (side force; placeholder chords), `WaveSample`, `constants.py`, `tests/test_method_fixes.py` (19 new tests), performance work (`np.cross`, closed-form rotation) ≈ 8.6 → 3.6 ms/step.
- Retuned: heave 40/6/10, pitch 1.0/0/0.1, roll 2.0/0/0.2 (was 15/1/5, 2/0/2, 3.5/0/2.5 with the wrong units). Takeoff 9.9 s → 2.9 s; 2 cm height settling 62 s → ~17 s; roll kick rejected in 0.7 s; 8° bank tracked; oblique-wave roll ≤ 1.3°.
- Verified: 42/42 tests; sweep results and robustness to doubled actuator delay/lag in doc 14 §H. Not re-run: randomised combined-input fuzz.
- Follow-ups: trim feed-forward; wing–tail downwash; flap section moment; real strut/hull/actuator measurements; merge or push `mujoco-viewer`.

### 2026-11-04 — Buoyancy cross-reference
- Compared the hull against Fossen ch.4 metacentric theory: roll stiffness was 23 % low and pitch 14× too low (no waterplane-inertia term). Replaced the lumped model with `HullHydrostatics` (footprint grid, full attitude kinematics, per-cell wave elevation); stiffness now within 1 % of theory (doc 15, E20).
- Added placeholder hull roll/pitch damping so the correctly stiff hull does not ring at rest; new params `hull_length`, `hull_beam`, `hull_x`, `hull_rot_damping` (`waterplane_area` now derived).
- 46 tests pass; foilborne metrics unchanged; gains did not need retuning. Everything remains local (no push), commits on branch `mujoco-viewer`.

### 2026-11-04 — Review packaging
- Added `tools/audit.py` (portable, reproducible: hydrostatics, linearize, checks, evaluate, sweep-angles, sweep-heave) and `docs/16-review-guide.md` (conventions, equations→code map, assumption ledger, evidence matrix, open questions, unexplained observations).
- Post-fix cruise values re-measured: trim flap 15.1°, thrust 17.1 N, pitch −0.30°; docs updated (the as-found doc 13 keeps the old figures).
- Open-loop poles of the corrected plant: +0.25 s⁻¹ lateral spiral-type, +0.029 s⁻¹ undiagnosed slow x-mode.

### Template for future entries
```
### YYYY-MM-DD — Title
- What changed (files)
- Why
- Verified how (test/run/sweep) — or "not verified"
- Parameter changes (old → new)
- Follow-ups
```
