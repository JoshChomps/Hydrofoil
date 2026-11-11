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

### 2026-11-05 — One app (MuJoCo) and the actuator drive model
- Folded the browser dashboard into the MuJoCo app: sliders (zero-gain actuators in the Control panel) for references, waves, live PID gains and drive settings; rolling plots via `set_figures`; keys and sliders share one value. Removed `dashboard.py`, fastapi/uvicorn, and `.claude/launch.json` (no web server left).
- Added `hydrofoil/actuation/`: thin-airfoil hinge coefficients (validated vs 2π, c/4, Glauert τ), quasi-steady Theodorsen damping and added inertia, moteus r4.11 + BLDC + gearbox electrical model, per-drive loads after the mixer. `tools/actuator_budget.py` runs 5 scenarios and a gear-ratio sweep.
- Findings (doc 17): the trimmed hinge moment is ≈ constant with speed (≈ 2.1–2.4 N m per main flap) while the moment per degree grows with V². 20:1 sits at the peak-current optimum, with ≈ 4 A rms / 19 A peak against 12 A / 100 A. The drive can make 29 N m, 4.5× the worst water load, so cap torque in firmware. The mixer sits ahead of the limits (a flap reached 35°). Flap effectiveness 0.21 conflicts with thin-airfoil 0.75.
- All local, no push.

### 2026-11-05 — Drives closed into the loop
- `actuation/servo.py`: per physical drive moteus PD + current/back-EMF/firmware-cap limits + gearbox + rotor/water/structure inertia + hinge moment, 1 ms sub-steps, mechanical stops; mixer moved ahead of the limits. Default `actuator_model = "servo"`; legacy `"kinematic"` kept. Optional q gain schedule (off).
- Flight metrics essentially unchanged with the gains as they were (doc 17 §7); a 4× weaker drive degrades gracefully; the limp-drive test confirms the fail-safe. The app shows the drives' own telemetry. 57 tests pass. Local only.

### 2026-11-06 — Audit verified against the original code
- New doc 18 and `tools/verify_original_audit.py`: every claim in docs 13/14/17 re-checked on commit 841d501 (original code) in an isolated interpreter.
- Confirmed: 7 errors (E1, E2, E3, E12, E13, E15, F1), 2 omissions (E7, E10), 2 latent errors (E4, E5), 4 minor (E8, E9, E11, E16); 5 items were not errors or not in the original.
- My claims corrected: E7 "energy not conserved" (wrong: νᵀCν = 0), E14 (AiryWave had `direction`; the original "Ki 0.05 blew up" comment is not reproducible), doc 13 W7 (no common limit; the opposite problem), the flap-effectiveness "conflict" (inboard half-span), E2's before-figures (8.5 s / 70.6 s).
- Found and fixed my own error: the hinge model used the full 0.6 m span for a 0.3 m inboard flap (`main_flap_span_frac`). Hinge loads and currents halved; the current-optimal ratio is ≈ 15:1 (10–20 flat); doc 17 refreshed; the viewer draws the flap inboard. 57 tests pass. Local only.

### 2026-11-09 — Trim feed-forward (roadmap Phase 0 item 2)
- `vehicle._trim_ff`: δ_ff = `trim_ff_deg` (15.1°) · (`trim_ff_speed` (4 m/s) / u)², clamped ±20°, added to the heave PID output; the heave PID's clamp is shifted by the FF so anti-windup judges the total flap command. `trim_ff_deg = 0` disables it. New params in `params.py`; test added.
- Verified (`tools/audit.py evaluate`, calm water): height-step 2 cm settling 8.6 s → 2.7 s; settling after takeoff 17.7 s → 12.2 s; takeoff 2.95 s, bank/roll/pitch metrics unchanged. 58 tests pass. Without the clamp shift the post-takeoff settling was worse (32 s: integrator wound up on top of the FF). Not re-run: wave cases, the doubled-delay robustness block.
- Follow-ups: the trim angle is a measured constant, not computed from W/(2qA·CLα·k); the remaining 12 s post-takeoff settle is the speed ramp (surge loop).

### 2026-11-09 — Wing–tail downwash and flap section moment (roadmap Phase 0 item 4, rest)
- Flap moment: `thin_airfoil_coeffs` now returns `cm_alpha` (≈ 0, AC at c/4) and `cm_delta` (−0.588/rad for a 40 % flap, equals Glauert −½ sinθ(1−cosθ)). `Foil(flap_cm_delta=…)` adds q·A·c·Cm_δ·δ to the pitch moment of each main half; scaled by `main_flap_span_frac` (0.5) and `flap_cm_factor` (0.8, same viscous placeholder as the hinge model). Switch: `flap_moment`.
- Downwash: tail AoA reduced by ε = `downwash_factor` · 2·CL_wing / (π·AR_wing), AR on the full 1.2 m span, using the main foils' corrected CL (`Foil.last_CL`). Quasi-steady: NO transport lag (l/V ≈ 0.3 s), and no strut/hull wake effect.
- Effect: cruise trim flap 15.1° → 16.1° (so `trim_ff_deg` 15.1 → 16.1), elevator −0.30° → −0.36°, thrust 17.1 → 18.0 N. Calm-water metrics: takeoff 2.95 → 3.10 s, post-takeoff 2 cm settle 12.2 → 14.6 s, height step 2.7 → 3.6 s, bank/roll/pitch unchanged, no touchdowns, wave roll unchanged.
- 60 tests pass (2 new: Glauert match, downwash/moment signs). Note: the "delay and lag doubled" block of `audit.py evaluate` printed identical numbers to the nominal block, so it does not appear to alter the servo-model path — not investigated.
- Follow-ups: transport-lagged downwash (affects short-period damping), retune if the lag is added, wing-wake on the elevator strut.

### 2026-11-09 — Downwash transport lag and drive-load consistency
- Tail downwash is now delayed by l/u (wing-to-tail distance / forward speed) from a history buffer in `VehiclePhysics` (`downwash_lag`, default on). The elevator drive's hinge-load flow (`actuation/loads.flow_models`) now subtracts the same downwash, so elevator hinge moment/current are consistent with the force model.
- Flight metrics barely change with the lag (z-settle 14.7 → 14.6 s, height step 3.58 → 3.55 s). `physics_rhs` called directly (audit linearize) sees the frozen last delayed value, so the linearised model omits the downwash dynamics.
- Drive budget re-run (`actuator_budget.py --model servo`, N = 20, cruise 4 and 6 m/s head seas), new model vs old (no flap moment/downwash, trim 15.1°): peak current 13.7 A and peak voltage 5.4 V unchanged (set by the takeoff step to the 20° stop); rms current 1.9 → 2.0 A (4 m/s) and 2.0 → 2.2 A (6 m/s); peak output torque 1.40 → 1.47 and 1.83 → 1.89 N m; elevator unchanged. The flap section moment reaches the drives only through the achieved flap angle; hinge loads scale with V². All within the moteus/BLDC limits. Not swept: other gear ratios with the new model.

### 2026-11-10 — 40 V / 25 A supply and rail-current check
- Supply is now 40 V from battery through the PDB to the motors, limited to 25 A (user-stated): `v_bus` 24 → 40 (params, `ControllerSpec`, `actuator_budget.py --vbus`), new `supply_i_max` = 25 A. The budget tool sums the three drives' bus power (time-aligned) into a rail current and prints it with a rough propulsion estimate thrust·u/η (η = 0.35 PLACEHOLDER) against the 25 A limit.
- Results (servo model, N = 20): actuators peak 3.1 A (mean 0.13 A); with propulsion ≈ 8.4 A at 4 m/s and 15.5 A at 6 m/s the rail totals 11.5 A / 18.6 A vs 25 A, all scenarios OK. Gear sweep (worst rail for actuators only): N = 5 → 2.8 A, 20 → 3.1 A, 40 → 11.4 A; peak phase current 13.8 A at N = 15, 14.1 A at 20, 27 A at N = 5 and 40; rate limit at N = 20 rises 2138 → 3564 deg/s. 60 tests pass.
- Caveats: the propulsion current is a guess (real motor/ESC efficiency, takeoff thrust, regen not modelled); the rail limit is checked in the budget tool, not enforced in the flight loop; actuator regen is not netted against propulsion.
- Not done: flight metrics re-measured at 40 V (tests pass; `audit.py evaluate` not re-run).

### 2026-11-10 — Robustness block of the audit fixed
- `audit.py evaluate`'s "delay and lag doubled" block printed the nominal numbers because it only set `actuator_params`, which only the kinematic actuator reads (the default is now the servo). It now flies the kinematic actuator nominal (20 ms delay/lag) and doubled (40 ms).
- Results: doubled delay/lag changes almost nothing (foilborne 3.05 → 3.06 s, 2 cm settle 18.84 → 18.87 s, roll 0.69 → 0.64 s, pitch 0.60 → 0.57 s, height step 3.85 → 3.83 s). Servo model for reference: 3.11 s / 14.6 s / 0.70 s / 0.62 s / 3.55 s. Wave cases not re-run in this block.
- Not done: flight metrics at 40 V were not isolated from this run's servo numbers beyond the above (they match the 24 V figures, so 40 V made no visible difference to flight).

### 2026-11-11 — Linearisation, LQR, sensors and Kalman estimator
- New: `linearize.py` (trim + Jacobian), `control/lqr.py` (scheduled MIMO LQR with integral action, PID takeoff with bumpless latched handover), `sensors/models.py`, `estimation/kalman.py`; params `controller` ("pid" default / "lqr") and `sensors` ("ideal" default / "noisy"). Doc 19.
- Verified: 67 tests pass (7 new). LQR vs PID on the audit scenarios: height step 3.55 → 1.18 s, pitch kick 0.62 → 0.21 s, wave roll (beam/oblique) 0.99/1.30° → 0.39/0.26°, bank height loss 17.7 → 4.8 mm, roll kick 0.70 → 0.63 s; bank settle 0.56 → 0.61 s. Roll weight tuned 2° → 0.8° (roll kick 1.38 → 0.62 s). With noisy sensors the boat still flies (3.3 mm rms height).
- Defaults unchanged (PID, ideal sensors). Trim finding: the PID's 16.1° flap at 4 m/s is the closed-loop value at θ = −0.37°; level trim is 14.1°.
- Not done: full evaluate table with noisy sensors beyond the quick check; LQR in the viewer UI; drive dynamics in the LQR model; rail-current enforcement.

### Template for future entries
```
### YYYY-MM-DD — Title
- What changed (files)
- Why
- Verified how (test/run/sweep) — or "not verified"
- Parameter changes (old → new)
- Follow-ups
```
