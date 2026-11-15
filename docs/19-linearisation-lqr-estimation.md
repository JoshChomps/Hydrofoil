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

## 2026-11-15 — Delay-aware LQR and main-foil incidence (supersedes the design and trim numbers above)

### Why
- **Delay.** The design above assumed the commanded flap is delivered instantly. With the servo's command delay raised from 10 ms to 30 ms the LQR limit-cycled in calm water (main flap 2.2° rms, height 4 mm rms; 13 mm at 80 ms) while the PID was unaffected. Linear check: the old gain is unstable on the drive + delay model from 20 ms at 4 m/s, and already at 10 ms at 7 m/s. Roll and pitch loops crossed over near the drive's own bandwidth (≈ 15 Hz).
- **Flap headroom.** Trim flap was 14.1° of a ±20° command limit at 4 m/s (24.2° at 3 m/s, above the limit), so the flap clipped at 20° in a 10 cm wave.

### What changed
- `main_incidence_deg` = 2.5 (new param, applied to both main-foil halves through `FoilGeometry.aoa_offset`). Trim flap: 11.9° at 3 m/s, 1.6° at 4, −3.5° at 5, −6.2° at 6, −7.8° at 7. `trim_ff_deg` 14.1 → 1.63; `_trim_ff` now schedules the wing's total angle of attack (incidence + 0.21 × flap) as 1/u².
- `control/lqr.py`: `augmented_model` (plant driven by the achieved surface angles + second-order drive model + chain of past commands, discretised at the control rate) and `design_delay_aware` (discrete LQR with a command-rate penalty, weights `du_*` = 250 °/s). `LQRController` feeds back the plant state, integrators, drive angles and rates (encoders) and its own last d commands. The old `design` is kept for reference.
- `lqr_delay_s` (new param): the loop latency the gains are designed for; `None` = the drive model's `delay_s`. On hardware, set it to the measured sensor-to-surface latency.

### Results
Gains designed for the nominal drive (14.7 Hz, 10 ms), real drive then altered; 3° pitch kick, then a 10 cm / 5 m head sea:
| Real drive | Calm-cruise flap rms | Kick height peak | Head-sea height rms |
|---|---|---|---|
| 14.7 Hz, 10 ms | 0.00° | 20.8 mm | 18.6 mm |
| 14.7 Hz, 30 ms | 0.00° | 23.1 mm | 20.0 mm |
| 14.7 Hz, 80 ms | 0.00° | 32.6 mm | 25.0 mm |
| 7.2 Hz, 40 ms | 0.00° | 28.7 mm | 23.3 mm |
| 4.7 Hz, 50 ms | 0.01° | 34.6 mm | 30.2 mm |

Linear delay margin of the default design (d = 1, `du` = 250): 130–150 ms across 3–7 m/s, but lightly damped far from the design delay (damping ratio ≈ 0.7 at 10 ms, 0.45 at 30 ms, 0.25 at 50 ms, 0.1 at 80 ms). A larger `du` gives higher gains and less margin (`du` = 3000 is unstable at 7 m/s).

`tools/audit.py evaluate`, servo drives, ideal sensors (old LQR in brackets): foilborne 1.74 s (3.11), 2 cm settle 6.2 s (13.2), height step 0.55 s (1.18), roll kick 0.73 s (0.63), 8° bank settle 0.72 s (0.61) with 3.8 mm height loss (4.8), pitch kick 0.23 s (0.21), beam / oblique wave roll 1.03° / 0.70° (0.39° / 0.26°).

Waves, LQR, ideal sensors, 5 m wavelength (old design and zero incidence in brackets):
| Case | Height rms | Flap range | Flap motor current rms |
|---|---|---|---|
| calm | 0 mm | 1.6° (14.1°) | 0.4 A (1.7) |
| head sea 10 cm | 18.5 mm (13.2) | −3.2…6.7° (8.5…19.7°, clipped) | 0.7 A (2.0) |
| head sea 15 cm | 27.8 mm (20.0) | −5.6…9.3° | 0.8 A (2.1) |
| following sea 10 cm | 40.9 mm (26.4) | −8.6…12.4° (5.8…19.7°, clipped) | 1.0 A (2.1) |
With noisy sensors the noise-driven flap activity dropped: peak flap rate 150 → 84 °/s, peak motor current 13.8 → 4.4 A in calm water.

### What it cost, and what is not covered
- Lower gains: height in waves is 40–55 % worse than the old design, and wave roll about 2.6× (still ≤ 1°). The old numbers were only reachable with a loop latency under about 20 ms.
- PID mode (servo drives): takeoff 2 cm settle 13.9 → 20.0 s with the incidence; height step unchanged (3.8 s). The legacy kinematic-actuator PID now shows a 12.6 s height step and 23 s settle in `evaluate`; not investigated.
- `audit.py`'s "head" wave is direction 0, i.e. a FOLLOWING sea (the worst case for height). True head seas (180°) are not in `evaluate` yet.
- Not tested: delay mismatch together with noisy sensors or with model error; backlash; delays above 80 ms. The Kalman filter still uses the model without the drive states. The height sensor still measures depth relative to still water, not the local wave surface.

## 2026-11-15 (later) — Realistic height sensing, heave filter, rider mass and CG shifts

### What changed
- **Height sensor sees the local surface** (`sensors/models.py`): the reading is the CG depth relative to the wave surface under the boat (z + wave elevation), as an ultrasonic / ToF / radar ranger gives. It used to be depth relative to still water, which no sensor measures. Channels now flag `fresh` samples (the value is held between samples).
- **Accelerometer channel** `az`: vertical (NED) acceleration of the CG, 200 Hz, 0.05 m/s² noise, bias random walk.
- **`HeaveFilter`** (`estimation/kalman.py`): kinematic filter, states [z, vz, accelerometer bias, wave (2)]. The accelerometer drives it, new height samples correct it, the sample's 50 ms age is modelled, and wave elevation is a band-pass noise state (`kf_wave` = {w0 4 rad/s, zeta 1.0, sigma 0.10 m}). It supplies height and heave velocity to the controllers; the model-based Kalman filter keeps attitude, rates and sway. It uses no mass, CG or foil data.
- **`cg_offset`** param and `VehiclePhysics.set_cg_offset((dx, dy))` (callable mid-flight): moves the CG relative to the foils and struts.
- **`audit.py evaluate`**: wave cases are now following (0°), head (180°), beam, oblique, each with height rms. The old "head" case was a following sea.

### Why the wave state is needed
With the realistic sensor and the range reading taken as height (wave state off), the boat chases every wave: in a 10 cm / 5 m head sea the flaps hit ±20°, flap rate 690 °/s peak, motor current 17 A peak, height peak error 212 mm.

### Results, LQR, noisy sensors, realistic height sensor (5 m wavelength unless noted)
| Case | Height rms | Flap range | Flap rate peak | Motor current peak |
|---|---|---|---|---|
| calm | 7.1 mm | −0.7…4.2° | 77 °/s | 4.8 A |
| head sea 5 cm | 10.2 mm | −2.7…6.9° | 92 °/s | 5.9 A |
| head sea 10 cm | 16.2 mm | −5.7…10.0° | 119 °/s | 6.5 A |
| head sea 15 cm | 22.9 mm | −8.8…13.1° | 148 °/s | 7.2 A |
| head sea 10 cm, 3 m | 10.4 mm | −4.0…8.5° | 134 °/s | 6.7 A |
| beam sea 10 cm | 13.2 mm | −9.2…12.5° | 88 °/s | 6.2 A |
| following sea 5 cm | 58.7 mm | −5.4…10.0° | 76 °/s | 5.8 A |
| following sea 10 cm | 114.8 mm | −14.2…16.4° | 93 °/s | 7.9 A |
No touchdown in any case; minimum foil depth below the local surface 0.33 m (following, 10 cm) against 0.55 m nominal.

`audit.py evaluate` with noisy sensors: foilborne 1.80 s, 2 cm settle 6.2 s, height step 0.54 s, roll kick 0.92 s, pitch kick 0.69 s, 8° bank 7.99° with 15.7 mm height loss; wave height rms (5 cm): head 11.0, beam 9.4, following 59.4, oblique (45°, quartering) 58.3 mm.

### Known weakness: following and quartering seas
The encounter frequency there is about 1.5 rad/s, too slow for the filter to tell a wave from a real height change, so the boat follows the surface and overshoots it (height rms above the wave's own 71 mm rms at 10 cm amplitude). Ideal sensors give 20 / 41 mm in the same cases. `HeaveFilter.bias_walk` trades this against calm-water drift: 0.002 (the sensor's own figure) gave 25 mm rms wander in calm water, 0.02 gives 7 mm but lets more slow wave through. Not tuned further.

### Rider mass and CG shifts (LQR designed on the nominal 25 kg boat, ideal sensors)
| Case | Result |
|---|---|
| CG ±5, ±10, ±15 cm fore/aft | flies, zero steady error; flap 3.5…−0.2°, elevator −2.1…2.5°; takeoff 1.9–2.9 s |
| CG 3, 6, 10 cm lateral | flies level; holds 1.2 / 2.3 / 3.8° of differential flap |
| mass ×1.3, CG +8 cm forward, +4 cm lateral | flies; flap 7.0°, takeoff 4.4 s |
| CG moved in flight, 10 cm aft | height peak 12.3 mm, pitch 0.6°, re-trimmed |
| CG moved in flight, 10 cm forward + 5 cm lateral | height peak 12.6 mm, roll 0.5°, re-trimmed |
| 100 kg (boat + 75 kg rider), 4 m/s, 50 N thrust | **does not take off**: level trim needs 45° of flap and 108 N |
| 100 kg, 8 m/s, 400 N thrust limit | flies on the nominal gains (68 N at cruise); CG ±10 cm and an in-flight shift of 10 cm aft + 5 cm lateral absorbed (height peak 18.9 mm); 10–36 mm steady height offset in some cases (integrator clamp) |

So the control law tolerates CG and moderate mass changes; a rider is a SIZING problem (foil area, speed, thrust), not a control problem. Everything above is on placeholder parameters. Not modelled: inertia change with the rider, thrust line below the CG, a rider who moves continuously, hull footprint shift.

### Not covered
Noisy sensors together with delay mismatch or model error; irregular seas; the height sensor's position (it is taken at the CG, a bow mount adds pitch coupling and preview); beam tilt errors; accelerometer misalignment.

### Is it good enough? (assessment, 2026-11-15)
- **Yes for holding flight through CG shifts**: the integral terms re-trim the flaps; no rider-position sensor is needed.
- **No for a rider on the current sizing**: every parameter describes a 25 kg unmanned model. At 100 kg the same foils need about twice the speed and four times the thrust. Foil area, cruise speed and thrust must be re-sized for the real all-up weight, then trim, gains and the estimator re-derived.
- **Weak in following and quartering seas** with noisy sensors (see above).
- **Open decision**: the real all-up weight (boat plus rider).

### Sensors required on the real boat
| Sensor | Used for | Notes |
|---|---|---|
| IMU (gyro + accelerometer), 200 Hz or more, with attitude solution | roll, pitch, rates; vertical acceleration driving `HeaveFilter` | the most important sensor; the height loop depends on it |
| Ride-height ranger (ultrasonic / ToF / radar) | slow correction of height | sees the local wave surface, not still water; low latency matters more than accuracy |
| Water-speed sensor (paddlewheel or pitot), GPS as backup | gain and trim scheduling | lift depends on speed through the water, which GPS does not give in a current |
| Flap and elevator position (moteus encoders) | the delay-aware LQR feeds back achieved angle and rate | already in the drive choice |
| Motor current and temperature (moteus telemetry), bus voltage/current | protection, jam detection | already available |

Worth adding: a second height sensor (a dropout or spray hit is then not fatal), and a bow mount for the height sensor so it sees waves before the foil does (untested: the model takes the sensor at the CG).

### Other findings to carry forward
- **Kinematic-actuator PID, 12.6 s height step**: caused mainly by `ki_pitch = 1.0`; the slow integrator tail sits at 26 mm against 16 mm around the 2 cm settle band. A settle-metric artefact, not a stability problem.
- **PID takeoff settle slowed from about 14 s to 20 s with the 2.5° incidence**; not diagnosed.
- **Why delay tolerance mattered**: the earlier LQR limit-cycled once actuator delay reached 30 ms (flap 2.2° rms in calm water). The delay-aware design is quiet out to 80 ms (linear margin 130–150 ms) and with drives of 4.7–14.7 Hz, at the cost of 40–55 % worse wave height holding than the old gains had on an ideal actuator.
- **Drive is oversized**: hinge moment about 1–1.5 N·m against 28.9 N·m peak; inertia-matched ratio about 4.6:1 against the 20:1 fitted.
- **Not modelled**: gearbox backlash, unsteady lift (Theodorsen), irregular seas, a measured free-surface factor, Reynolds-number mismatch of the polar. The roll arm may be overstated if the flaps are inboard.
- **Literature**: the MDPI paper first cited (Modelling 7(2):51) is a PID-LQR scheme for a full-bridge boost converter, simulation only; the LQI here already contains what it does.
