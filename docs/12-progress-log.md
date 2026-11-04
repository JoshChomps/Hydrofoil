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
| Cascade PID (heave/pitch/roll) + surge | done, **not retuned** after corrections |
| Actuator model | done, placeholder numbers |
| Dashboard | done |
| Unity bridge + pilot uplink | done |
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
- Physics files were not changed.

### Template for future entries
```
### YYYY-MM-DD — Title
- What changed (files)
- Why
- Verified how (test/run/sweep) — or "not verified"
- Parameter changes (old → new)
- Follow-ups
```
