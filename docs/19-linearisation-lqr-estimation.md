# 19 — Linearisation, LQR, sensors and estimation

Added 2026-11-11 (roadmap Phase 3 and "State estimation and sensors"). All numbers are on the **placeholder** vehicle; when real
parameters/CAD arrive the gains re-derive automatically (`design()` is called at construction), but the weights need re-checking.

## Modules
| File | What |
|---|---|
| `hydrofoil/linearize.py` | `trim(vp, speed)` solves du/dt = dw/dt = dq/dt = 0 for (δ_main, δ_elev, thrust) at fixed θ (Newton); `linearize` central-difference A (12×12), B (12×4); `reduced` drops x, y, ψ. Quasi-steady downwash is included, the transport lag is not (hidden delay state). |
| `hydrofoil/control/lqr.py` | Speed-scheduled LQR on the three flap channels with integral action on height, roll, pitch. Gains at 3–7 m/s, interpolated in u. PID flies takeoff; handover at u ≥ 3.2 m/s and height error < 0.15 m, latched, bumpless on the heave channel. Select with `params["controller"] = "lqr"` (needs the servo actuator model). |
| `hydrofoil/sensors/models.py` | Gyro (200 Hz, 0.2 °/s + bias walk), AHRS attitude (100 Hz, 0.3°), height (20 Hz, 1 cm, 50 ms latency, 2 % dropout), speed (10 Hz, 0.1 m/s). All placeholders. |
| `hydrofoil/estimation/kalman.py` | Scheduled steady-state Kalman filter on the linearised model (8 states: z φ θ v w p q r), discretised at the control rate, fed with the achieved flap angles; skips missing sensors. Select with `params["sensors"] = "noisy"`. |

## Trim and open-loop plant (level, θ = 0, h = 0.55 m)
Trim flap δ_main: 24.2° at 3 m/s, 14.1° at 4, 9.0° at 5, 6.2° at 6 (lift ∝ u²); thrust 17.2 / 17.6 / 21.6 / 28.1 N; elevator −0.4° at 4 m/s.
The closed-loop PID sits at θ = −0.37° (proportional-only pitch loop) and therefore needs 16.1° at 4 m/s: the trim is a family in θ, not a bug.
Open-loop poles at 4 m/s: fast stable modes (−129 … −19 s⁻¹), a lightly damped pair −0.19 ± 0.65j, −0.03, and **+0.247 s⁻¹ (lateral, spiral-type)**.
Fully controllable (rank 9/9) from the three flaps. The +0.029 s⁻¹ "x-mode" in doc 16 was the unconstrained surge/position direction and does not appear in the reduced model.

## LQR design
State [z φ θ v w p q r ∫z ∫φ ∫θ], inputs [δ_main δ_elev δ_roll] in degrees, Bryson weights (1 / excursion²): z 3 cm, φ 0.8°, θ 2°, v 0.3 m/s, w 0.15 m/s, p, q 8 °/s, r 15 °/s, integrators as the angles, inputs 8°.
The roll weight was tuned: with φ at 2° the roll kick settled in 1.38 s; at 0.8° it settles in 0.62 s. Weights live in `DEFAULT_WEIGHTS`; override with `params["lqr_weights"]`.
Surge is **not** in the LQR (u is a scheduling parameter; thrust stays on the PID).

## Results (calm water + waves; `tools/audit.py evaluate`, servo drives, ideal sensors)
| Metric | PID | LQR |
|---|---|---|
| takeoff (foilborne) | 3.11 s | 3.11 s |
| 2 cm height settle after takeoff | 14.6 s | 13.2 s |
| 2 cm height step | 3.55 s | 1.18 s |
| 5° roll kick to 0.5° | 0.70 s | 0.63 s |
| 8° bank: final / settle 1° | 8.46° / 0.56 s | 8.00° / 0.61 s |
| 8° bank height loss | 17.7 mm | 4.8 mm |
| 3° pitch kick to 0.5° | 0.62 s | 0.21 s |
| beam / oblique wave max roll | 0.99° / 1.30° | 0.39° / 0.26° |

Noisy sensors + Kalman estimator, quick check (LQR, 10 s of cruise after settling): true-state rms z 3.3 mm, φ 0.024°, θ 0.067°; main-flap command jitter 1.1° rms (noise driven); height-estimate error ≈ 1 mm. Full disturbance table with noisy sensors: see the progress log if present.

## Limits and open items
- The linear model ignores the downwash lag, the drive dynamics (the LQR assumes the commanded flap is delivered) and surge coupling; it works here because the drives are fast and the speed loop slow. A model-based design that includes the drive would be the next step.
- The Kalman filter uses fixed placeholder process noise, treats the latent height sample as current, and has no bias states (the gyro bias walk is unmodelled).
- Only the servo actuator model is supported with the LQR. Thrust is still PID; the 25 A rail limit is not enforced.
- LQR is not selectable in the viewer UI; set `params["controller"]` / `params["sensors"]` before `VehiclePhysics(...)`.
- Re-tune weights (and re-run `evaluate`) after real parameters arrive; the scheduling speeds (3–7 m/s) must cover the operating envelope.

### Noisy sensors: estimator retune
The first noisy evaluation showed a 3° pitch kick settling in 4.36 s (ideal: 0.21 s). Cause: the Kalman process noise on z, φ, θ was 1e-4, so the filter rejected a real attitude change for seconds. Retuned to z 3e-3, φ/θ 5e-3 (kick 0.30 s, elevator jitter 1.1° → 0.5°). Larger values (0.02+) track faster but leak sensor noise into the flaps (jitter 1.1–5°); 0.05 even made the 0.5° settle metric fail through noise. Values live in `kalman.py` (`kf_process` overrides).

## PID vs LQR, retuned estimator (full audit scenarios)
| Metric | PID ideal | PID noisy | LQR ideal | LQR noisy |
|---|---|---|---|---|
| takeoff | 3.11 s | 3.14 s | 3.11 s | 3.14 s |
| 2 cm settle after takeoff | 14.6 s | 15.0 s | 13.2 s | 14.3 s |
| 2 cm height step | 3.55 s | 4.3 s | 1.18 s | 1.41 s |
| 5° roll kick to 0.5° | 0.70 s | 0.68 s | 0.63 s | 0.31 s |
| 3° pitch kick to 0.5° | 0.62 s | 0.59 s | 0.21 s | 0.29 s |
| 8° bank final / settle | 8.46° / 0.56 s | 8.44° / 0.56 s | 8.00° / 0.61 s | 7.96° / 0.53 s |
| 8° bank height loss | 17.7 mm | 28.4 mm | 4.8 mm | 9.1 mm |
| beam / oblique wave roll | 0.99 / 1.30° | 1.17 / 1.49° | 0.39 / 0.26° | 0.46 / 0.40° |
No touchdowns in any case. Noisy sensors cost the PID more than the LQR; the noisy LQR beats the ideal-sensor PID on height step, pitch kick, bank height loss and wave roll.

## Robustness to model error (ideal sensors; LQR designed on the NOMINAL plant, nothing retuned)
| Plant | PID settle / step / bank loss | LQR settle / step / bank loss |
|---|---|---|
| nominal | 14.6 s / 3.55 s / 17.7 mm | 13.2 s / 1.18 s / 4.8 mm |
| mass and inertia ×1.3 | never / 15.4 s / 130 mm | 43.6 s / 1.17 s / 9.2 mm |
| main chord −13 % | 13.4 s / 2.95 s / 16.1 mm | 12.7 s / 1.20 s / 5.4 mm |
| main foil 8 cm forward | 23.1 s / 2.84 s / 15.3 mm | 7.5 s / 1.05 s / 4.3 mm |
| tail arm −25 % | never / 12.4 s / 96 mm | 31.3 s / 0.49 s / 10.0 mm |
("settle" = 2 cm height after takeoff within 60 s; "never" = not inside 2 cm in 60 s.) The LQR is equal or better in every row except bank settling on the heavy/chord cases (0.75 / 0.64 s vs 0.56 / 0.55 s) and the heavy roll kick (0.71 vs 0.70 s). Not tested: perturbations beyond these (e.g. much heavier, different foil lift slope), combined perturbations, or sensor noise together with model error. The PID's feed-forward trim angle is also a nominal-plant number, which partly explains its weaker showing.

## Default controller: LQR (decided 2026-11-11)
`params["controller"]` now defaults to `"lqr"`; `"pid"` remains selectable. If the actuator model is not `"servo"` the vehicle warns and falls back to the PID. The PID still flies takeoff and its gains (and the viewer's live PID gain sliders) only matter there / in PID mode. Sensors default to `"ideal"`. Constructing a vehicle now costs ≈ 3.7 s (gain design at 5 speeds).

## Limits and open items
- The linear model ignores the downwash lag, the drive dynamics (the LQR assumes the commanded flap is delivered) and surge coupling; it works here because the drives are fast and the speed loop slow. A model-based design that includes the drive would be the next step.
- The Kalman filter uses fixed placeholder process noise, treats the latent height sample as current, and has no bias states (the gyro bias walk is unmodelled).
- Only the servo actuator model is supported with the LQR. Thrust is still PID; the 25 A rail limit is not enforced.
- LQR is not selectable in the viewer UI; set `params["controller"]` / `params["sensors"]` before `VehiclePhysics(...)`.
- Re-tune weights (and re-run `evaluate`) after real parameters arrive; the scheduling speeds (3–7 m/s) must cover the operating envelope.

Full evaluation with noisy sensors + Kalman estimator (same scenarios, true-state metrics):
| Metric | LQR ideal | LQR noisy |
|---|---|---|
| takeoff / 2 cm settle | 3.11 / 13.2 s | 2.92 / 14.6 s |
| 2 cm height step | 1.18 s | 1.58 s |
| 5° roll kick to 0.5° | 0.63 s | 0.68 s |
| 8° bank: final / settle / height loss | 8.00° / 0.61 s / 4.8 mm | 7.95° / 0.23 s / 15.3 mm |
| **3° pitch kick to 0.5°** | 0.21 s | **4.36 s** |
| head / beam / oblique wave roll | 0 / 0.39 / 0.26° | 0.13 / 0.51 / 0.56° |
Nothing touches down. The pitch-kick settling is the one real degradation and is **not diagnosed** (suspects: estimator pitch dynamics after a state jump the model did not predict, and the 0.3° attitude noise against the 0.5° band). Everything else degrades mildly.
