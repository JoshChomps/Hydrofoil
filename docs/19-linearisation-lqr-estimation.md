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
