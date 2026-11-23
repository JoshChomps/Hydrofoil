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

## 2026-11-15 (later) — Manned sizing, 250–300 kg

All-up weight given as about 250 kg, up to 300 kg (boat plus rider). New preset `MANNED_PARAMS` (`params.py`); `DEFAULT_PARAMS` (25 kg model) is unchanged.

### Sizing
| | Model (`DEFAULT_PARAMS`) | Manned (`MANNED_PARAMS`) |
|---|---|---|
| Mass | 25 kg | 275 kg nominal, tested 250 and 300 |
| Main foil | 2 × 0.15 × 0.6 m = 0.18 m², 2.5° incidence | 2 × 0.28 × 1.1 m = 0.62 m², 3° incidence, span about 2.3 m |
| Elevator | 0.10 × 0.3 m, 1.2 m aft | 0.17 × 0.5 m, 2.0 m aft |
| Cruise speed / hull clears the water | 4 m/s / about 3 m/s | 8 m/s / 5.6–6.3 m/s (250–300 kg) |
| CG | between hull and foil | in the hull, 0.35 m above the hull bottom (`hull_offset` −0.35) |
| Foil depth / hull clearance flying | 0.55 / 0.55 m | 0.9 / 0.5 m (1.4 m struts) |
| Hull | 2.0 × 0.30 m | 3.6 × 1.2 m |
| Thrust limit | 50 N | 1000 N (600 N also tested) |
| Drive torque cap | 10 N·m | 28 N·m (same mj5208 + 20:1 drive) |

Inertias, hull drag and damping, thrust and surface inertias are PLACEHOLDERS scaled from the model.

How the foil was chosen: trim-flap scan over area (0.40, 0.50, 0.62 m²) and incidence (2–4°) at 250 and 300 kg, 4–10 m/s (flap effectiveness is only 0.21, so the incidence carries the weight and the flap trims). At 0.62 m² and 3° the level-trim flap is +8.4° (300 kg) at 6 m/s, −1.7° / −3.9° (300 / 250 kg) at 8 m/s, −6.4° at 10 m/s: within ±12° of the ±20° limit across the whole speed range. Smaller foils, 300 kg: 0.50 m² at 3° needed 25° of flap at 5 m/s, 0.40 m² needed 18° at 6 m/s. Cruise thrust about 190 N (≈ 1.5 kW at the propeller at 8 m/s).

### Code changes
- `control/lqr.py`: the drive model in the LQR design now includes the restoring hydrodynamic hinge stiffness (`hinge_stiffness`; see "Verification" below).
- `lqr_speeds`, `lqr_engage_speed`, `est_engage_speed` params (were hard-coded 3–7 m/s, 3.2 m/s, 3.0 m/s); `kf_heave` passes `HeaveFilter` settings.
- The LQR and the model-based Kalman filter are now designed on the NOMINAL CG (`cg_offset` = 0): a controller cannot know where the rider sits. Designing on the shifted plant also failed outright for a 25 cm forward CG (no trim at 4.5 m/s).
- `linearize.trim`: thrust Newton step clipped to ±200 N (was ±5, which capped thrust at about 215 N after 40 iterations, so heavier boats never converged).

### Results (LQR, 275 kg unless noted)
Calm water, takeoff from rest:
| Case | Foilborne | 2 cm settle | Cruise flap |
|---|---|---|---|
| 250 kg, 1000 N | 1.8 s | 4.1 s | −3.9° |
| 250 kg, 600 N | 3.2 s | 5.5 s | −3.9° |
| 300 kg, 1000 N | 2.5 s | 5.1 s | −1.7° |
| 300 kg, 600 N | 4.6 s | 7.2 s | −1.7° |
| 275 kg, noisy sensors | 2.2 s | 13.7 s | height 7.2 mm rms |

Waves, 15 cm / 10 m unless noted, height rms / peak [mm]:
| Case | Ideal sensors | Noisy sensors |
|---|---|---|
| head | 26 / 37 | 25 / 58 |
| following | 37 / 54 | 66 / 125 |
| beam | 22 / 31 | 21 / 52 |
| oblique 45° | 28 / 43 | 77 / 140 |
| head 25 cm / 15 m | 47 / 68 | 31 / 68 |
| following 25 cm / 15 m | 46 / 66 | 118 / 213 |
No touchdown in any case; flap within −9.3…+3.7°, elevator ±5°, roll channel ±4°.

Rider CG (ideal sensors, 250 and 300 kg): CG ±15 and ±25 cm fore/aft, 10 and 20 cm lateral at rest all take off and hold height to < 1 mm. In-flight shifts at 300 kg: 20 cm aft + 10 cm lateral, height peak 26 mm; 20 cm forward + 15 cm lateral, height peak 31 mm, roll peak 0.9°; both re-trim to zero (noisy sensors: 32 mm peak, 7.5 mm offset at the end of the run).

### Findings
- **Drive torque**: with the model's 10 N·m firmware cap the boat rolled over on takeoff with a 10 cm lateral CG. Hinge moments on the larger flap reach 12–13 N·m at takeoff; the low-side flap stalled short of its command and the roll channel ran out. With 28 N·m (just under the mj5208 × 20:1 peak of about 29 N·m) the same case flies; torque limiting falls from 15–97 % of steps to 0.1–2.6 %. Cruise hinge moment 1–5 N·m calm, 7–12 N·m in waves; the drive has to be run close to its peak, so the gearbox rating (30 N·m placeholder) and the rail current (three drives against the 25 A supply limit) need checking. The earlier "drive is oversized" finding applies only to the 25 kg model.
- **Following and quartering seas** are worse at 8 m/s: encounter frequency in 10–15 m following and quartering seas is about 1–1.5 rad/s, where wave and real height change look the same to the range sensor. Heave-filter sweep (noisy sensors; calm / following 15 cm 10 m / oblique / following 25 cm 15 m / head, rms mm): default 6 / 90 / 183 / 259 / 19; chosen `kf_wave` w0 2 rad/s, sigma 0.15 m, `bias_walk` 0.005: 13 / 67 / 79 / 118 / 22; slower still (bias_walk 0.002) 25 / 64 / 61 / 92 / 32. Tuning only trades calm-water wander against following-sea error.
- **What would fix it**: an absolute vertical reference that does not see the waves. GNSS Doppler vertical velocity (a few cm/s, no drift) fused into `HeaveFilter`, or a lower-drift IMU (tactical-grade bias stability). Not yet modelled; this is now the main gap for a manned boat. A bow-mounted second height sensor (preview) is the other option.
- The high CG (1.75 m above the foil) is handled by the same controller without retuning; no stability issue found.

### Not covered
Inertias and hull drag hump are placeholders; no propeller/motor model (thrust is a clamped PID); a rider moving continuously; irregular seas; turns at 8 m/s.

### Verification (second pass, 2026-11-15)
- **Hand check of trim**: CL needed at 8 m/s, 275 kg is 0.134, i.e. 2.3° effective AoA; with the 0.61 Helmbold and 0.87 free-surface factors that is −3.3° of flap by hand against −2.8° from `trim` (the difference is strut lift and the flap pitching moment). Thrust/weight 0.071, foil L/D about 20.
- **Numerical**: dt 0.005 against 0.01 and sensor seeds 0/1/2 change height rms by under 10 % (calm noisy 11–17 mm rms over the first 30 s including settling). The fictitious `heave_damping` (600 N s/m, 7 % of the foils' own 8400 N s/m) and adding the unmodelled roll added inertia to `ixx` change nothing material.
- **Linear delay margin** (true loop delay at which the closed loop goes unstable, design for 10 ms): model 150 ms; manned 120 ms at 5.5 m/s, 80 ms at 8 and 9.5 m/s. Half the model's, still well above the 10 ms loop.
- **Correction**: the hull clears the water at 5.6–6.3 m/s (250–300 kg; 1000 or 600 N), not "about 5 m/s" as first written.

### What changes with weight beyond the numbers
| Issue | Model (25 kg) | Manned (275 kg) | Consequence |
|---|---|---|---|
| Hydrodynamic hinge stiffness vs servo kp 250 N m/rad | 4 N m/rad at 4 m/s (2 %) | 98 at 8 m/s, 153 at 10 m/s (40–60 %) | the flap stops short of its command by k/(kp+k), 28 % at 8 m/s; the LQR's drive model ignored it (fixed). Recommend kp ≥ 600 N m/rad (tested: slightly better everywhere) |
| Drive torque | 1–1.5 N m of 29 | 7–12 N m in waves, 24–33 N m peaks | drive runs near its peak; torque-limited on 3–4 % of steps; check gearbox rating and supply current |
| Drive bandwidth | 14.7 Hz | 10.6 Hz flap, 9.5 Hz elevator | surface + water inertia now 51 % of the flap's total (was 4 %) |
| Open-loop unstable heave/pitch mode | +0.25 1/s (doubles in 2.8 s) | +0.64 1/s (doubles in 1.1 s) | faster departure when control is lost |
| Control power per degree | pitch 112, roll 182 deg/s² | pitch 58, roll 104 deg/s² | about half the authority per degree |
| Delay margin | 150 ms | 80 ms | see above |
| Reynolds number vs the XFLR5 polar (2.7e6) | 5.7e5 (polar not valid) | 2.1e6 (close) | the polar is now appropriate |
| Cavitation number | 12.8 | 3.3 at 8 m/s, 2.1 at 10 m/s | flap-hinge and tip suction peaks need checking above 10 m/s or at large flap in waves |
| Heave added mass (strip theory) | 0.97 × mass | 0.55 × mass | 2-D strip theory overstates this for AR 4 panels |
| Roll added inertia (not modelled) | 1.7 × ixx | 0.8 × ixx | tested: little effect, but should be in the model |
| Foil root bending per half | 43 N m at 1 g | 810 N m at 1 g, 2.4 kN m at 3 g | structural design case (foil root, strut, flap hinge mounts) |
| Vertical acceleration at the rider | — | 0.14 g rms in 25 cm head seas (ideal sensors) | about 1.4 m/s², "uncomfortable" on the ISO 2631-1 scale; ride comfort becomes a design objective |

### Failure cases at 8 m/s cruise (calm water, failure at 15 s)
| Failure | Manned (275 kg) | Model (25 kg) |
|---|---|---|
| Thrust cut | settles stern first after 8.1 s: 2.4 m/s, 0.7 m/s sink, pitch +6°, level | similar (4.9 s) |
| One main flap drive loses power (limp) | no effect: the hinge stiffness centres the flap near trim | rolls to 54° (stiffness too low to centre it) |
| One main flap jammed at trim (−2.8°) | no effect | — |
| One main flap jammed at +8, +15 or −12° | **capsizes**: roll above 50° within 2–5 s, hull hits at 7.8–8 m/s | — |
| Elevator drive loses power | **nose-in**: pitch −13°, hull hits in 1.1 s at 8.2 m/s, 1 m/s sink | — |
| Height sensor lost (noisy sensors) | drifts 0.28 m high over 20 s; no warning | — |
| Speed sensor stuck | no effect (overspeed to 8.6 m/s) | — |

With a person aboard these are the most important findings. The controller has no fault handling:
- A jammed flap is survivable in principle: the working flap matches the jammed one and pitch or speed trims the total lift (+8° is the trim flap at about 5.8 m/s for 275 kg). That needs jam detection (encoder against command) and a reconfigured law.
- Loss of the elevator has no control-surface cure on this layout. It needs a drive that fails to a safe angle (spring return or brake to a slight nose-up angle), a redundant elevator drive, or an immediate thrust cut.
- Loss of the height sensor needs detection and a commanded slow-down to touchdown.
- None of this is in the code yet.

## 2026-11-15 (later) — Operating protocol: supervisor for takeoff, cruise, landing and faults

Goal: get onto the foils for the energy saving (any small hull clearance will do), cruise safely at a speed the pilot chooses, and come back to displacement running safely, including when something fails. Implemented as `hydrofoil/control/supervisor.py`, enabled with `params["supervisor"] = True` (or a dict of overrides); off by default, so nothing else changes. Manned boat (`MANNED_PARAMS`, 275 kg).

### Why foil: power against speed (275 kg, calm)
| Speed | Displacement (hull in, flaps neutral) | Foiling (0.1–0.5 m clearance) |
|---|---|---|
| 3 m/s | 0.79 kW | — |
| 4 m/s | 1.92 kW | — |
| 5 m/s | 3.86 kW | 1.0 kW (trim flap +13–15°, too close to the limit) |
| 6 m/s | above 5.45 m/s not reachable with 1000 N | 1.1 kW |
| 7 m/s | — | 1.25–1.31 kW |
| 8 m/s | — | 1.52–1.63 kW |
| 9 / 10 m/s | — | 2.0 / 2.5 kW |
Propeller power (thrust × speed). Foiling needs less than half the power above about 4.5 m/s; the cheapest cruise is 6–7 m/s. Hull clearance changes power by only about 5 % between 0.1 and 0.5 m, so flying low costs nothing. The hull drag is a placeholder (`120 × wetted fraction × u²`), so the displacement column is indicative only.

### Operating envelope (defaults in `DEFAULT_SUPERVISOR`)
| Item | Value | Why |
|---|---|---|
| Displacement speed cap `v_disp_max` | 4 m/s | the hull hump makes faster displacement running wasteful |
| Liftoff `v_liftoff` / takeoff speed `v_takeoff` | 5.5 / 6.5 m/s | hull clears at 5.6–6.3 m/s; trim flap +13° at 5 m/s |
| Foiling speed floor `v_min_foil` | 6.0 m/s | trim flap about +7°: margin both ways for waves and control |
| Speed ceiling `v_max` | 9.5 m/s | cavitation number about 2.3; trim flap about −7° |
| Hull clearance | default 0.25 m; `h_min` 0.15 m + 1.2 × estimated wave amplitude; `h_max` 0.6 m | flying low costs nothing; waves need margin |
| Ramps | climb 0.10 m/s, sink 0.08 m/s, speed 0.5 m/s² | gentle for a rider |
| Bank | ±8° while foiling, 0 otherwise | existing verified cap |
| Thrust | 0 … max (no reverse thrust) | a propeller gives no useful reverse thrust; the old surge PID could command −1000 N |

### Modes
DISPLACEMENT → (pilot: foil) TAKEOFF → (hull clear, at height) FOILING → (pilot: land) LANDING → (hull in the water) DISPLACEMENT, plus EMERGENCY_LANDING from any flying mode on a fault.
- **DISPLACEMENT**: main flaps held at `disp_flap_deg` (0, no lift wanted); the elevator and differential flap keep the PID pitch / roll loops for damping. Speed ≤ `v_disp_max`.
- **TAKEOFF**: speed ref ramps to `v_takeoff`; the height ref follows the hull until `v_liftoff`, then ramps up at `climb_rate`. The LQR is held off until the hull is clear (it is designed for the foilborne plant), the PID flies the lift-off. Aborts to LANDING if the pilot cancels or `v_liftoff` is not reached in 25 s.
- **FOILING**: pilot speed clamped to [v_min_foil, v_max], height to [h_min + wave margin, h_max], both rate-limited.
- **LANDING**: height ref ramps down at `sink_rate` with the speed held at or above `v_min_foil` (the foils keep their authority until the hull is down), then speed ref to the pilot's displacement speed, controllers reset (the LQR handover re-latches on the next takeoff).
- **EMERGENCY_LANDING**: see faults. A latched fault blocks the next takeoff.

The supervisor only uses what the boat can measure: the controllers' estimated state, the drive encoders and telemetry, the hinge model, and the sensors' fresh-sample flags.

### Fault detection
Drive residual = |encoder − command − M_hinge / kp|, i.e. the error left after the static deflection the hinge load explains (with kp 250 N m/rad a healthy flap under 17 N m sits 4° off its command; without this correction the checks false-alarmed in 25 cm seas). A drive is declared lost when the residual exceeds 4° and the encoder has not moved 0.2° in 0.2 s (jam), or the residual lasts 0.3 s (elevator 0.15 s) while the drive is not torque-limited (power loss), or 1.5 s regardless. Height or speed sensor: no fresh sample for 0.5 s. Attitude: roll > 15° or pitch > 8° while flying.

### Fault responses and results (8 m/s plan: displacement 3 m/s, foil at 10 s with 7 m/s, fault at 30 s; calm unless noted)
| Fault | Response | Without supervisor (docs/19 failure table) | With supervisor |
|---|---|---|---|
| Flap jammed at +8° | healthy flap matches the jammed one ± the roll command (`mix`), thrust 0, height ref down | roll > 50°, capsize | detected 0.19 s; roll pk 1.2°; touchdown 5.3 m/s, 0.10 m/s sink, level |
| Flap jammed at +15° | same | capsize | roll pk 3.0°; touchdown 4.9 m/s, 0.10 m/s sink |
| Flap jammed at −12° | same | capsize | roll pk 1.8°; touchdown 6.2 m/s, 0.09 m/s sink |
| Flap jammed at trim | same | no effect | lands normally |
| Flap jam during takeoff (+8°) / during landing | same | — | lands, roll ≤ 1.9° |
| Flap jam +8°, 15 cm head sea, noisy sensors | same | — | roll pk 1.5°; touchdown sink ≤ 0.06 m/s |
| Elevator drive power loss | thrust cut, height ref down fast | pitch −13°, hull hits at 8.2 m/s, 1 m/s sink, in 1.1 s | detected 0.15 s; pitch −4.3°; touchdown 7.0 m/s, 0.56 m/s sink |
| Elevator loss with a fail-safe drive (spring return to 0°, 150 N m/rad) | same | — | pitch within −1.1…+2°; touchdown 6.8 m/s, 0.28–0.37 m/s sink |
| Elevator loss, 15 cm head sea | same | — | touchdown 7.1 m/s, 0.65 m/s sink, pitch −5° |
| Height sensor lost (noisy) | normal landing profile on the IMU-propagated height | drifts 0.28 m high, no warning | lands after 8 s; 0.27 m/s sink |
| Speed sensor lost (noisy) | landing profile, then thrust 0 | no effect | lands; 0.01 m/s sink |

### Normal missions (noisy sensors unless noted, 100 s: 10 s displacement, takeoff, 60 s foiling at 7 → 8.5 → 6.5 m/s, landing)
| Sea | Energy (prop) | Foiling clearance min…max | Vertical accel rms | Landing |
|---|---|---|---|---|
| calm (ideal sensors) | 30.9 Wh | 0.23…0.27 m | 0.003 g | 6.0 m/s, 0.11 m/s sink, level |
| calm | 30.9 Wh | 0.21…0.48 m | 0.012 g | 0.09 m/s sink |
| 15 cm / 10 m head | 30.8 Wh | 0.25…0.53 m | 0.078 g | first contact 0.02 m/s; then wave slaps up to 0.66 m/s sink while slowing through 5.7–4.7 m/s |
| 15 cm / 10 m following | 35.6 Wh | 0.24…0.65 m | 0.037 g | 0.06–0.23 m/s sink |
| 15 cm / 10 m oblique 45° | 31.5 Wh | 0.03…0.62 m | 0.017 g | 0.21 m/s sink |
| 25 cm / 15 m head | 33.9 Wh | 0.35…0.64 m | 0.068 g | first contact 0.02 m/s; slaps up to 0.49 m/s while slowing |
| 25 cm / 15 m following (75 s run) | — | 0.22…0.79 m | 0.023 g | not reached in the run |
No hull contact while foiling in any case; roll ≤ 0.7° while foiling. The wave margin raises the clearance automatically (0.25 → up to about 0.5 m).

### Findings and limits
- **Wave-height estimate**: taken from the raw ranging-sensor reading (√2 × its 10 s standard deviation), because the heave filter's wave state read only 0.07 m in a 0.25 m following sea. It includes the boat's own motion, so it errs high; in the oblique sea the minimum clearance still fell to 0.03 m.
- **Coming off the foils in head seas** is gentle at first contact, but the hull then takes wave slaps (up to 0.5–0.7 m/s) while it slows through the hump at 5–4.5 m/s; slowing faster once down, or landing down-sea, would shorten this.
- **Takeoff in waves** touches the hull several times between 4.5 and 7 m/s (up to 0.8 m/s sink in 25 cm head seas). Accelerating through the hump in waves is the roughest phase; a faster liftoff (higher thrust) or taking off down-sea would help. Not modelled with any credibility: the hull is wall-sided.
- **Low-speed displacement in waves with no elevator** (after an elevator loss in a head sea) pitched to 25° at 1–2 m/s in the sim: hull seakeeping is a placeholder, but it says a drifting, unpowered boat needs its own damping or a sea anchor.
- **Elevator loss remains the worst case**: the supervisor halves the nose-down and the sink rate, but there is no surface that can replace the elevator. A spring-return (fail-safe) elevator drive halves it again. This is the main hardware change to make.

### What to change on the real boat
1. Fail-safe elevator drive: spring return (or a brake) to 0°, or a second elevator drive; cut thrust on elevator loss.
2. Drive firmware must report encoder position and torque-limit state at the control rate (the jam check needs both) and the flight computer needs the hinge model to separate a heavy load from a jam.
3. Keep the foiling speed floor of 6 m/s and the ceiling of about 9.5 m/s; cruise 6–7 m/s for range.
4. A ranging sensor with a clean 20 Hz output (wave estimate) and the IMU (it carries the landing when the ranger fails).
5. No reverse thrust in the surge loop; throttle-down rather than braking.

Not covered: turns, irregular seas, rider moving during takeoff/landing, propeller/motor dynamics, battery limits.

## 2026-11-15 (later) — Autotuner, flight observer, boat editor

### Autotuner (`hydrofoil/tuning/`)
`scoring.py` flies a parameter set through five scenarios that scale with the boat (wave amplitude 0.3 x flying height, wavelength 3 x hull length, CG shift 5 % / 3 % of hull length): takeoff from rest, head sea, following sea, rider moving, and a head sea on the noisy sensors and estimators. `cost` folds height rms and peaks, overshoot, settling, flap rate, flap saturation and drive limiting into one number; failures (touchdown, |roll| > 30 deg, never foilborne) add 100.

`autotune.py` runs a staged loop, writing progress to JSON after every step:
1. the current values;
2. classical methods (`classical.py`):
   - PID: a relay feedback test on each loop in the sim (Astrom and Hagglund, Automatica 20(5), 1984) gives the ultimate gain and period, then the rule sets Ziegler-Nichols classic, "some overshoot" and "no overshoot" (Ziegler and Nichols, Trans. ASME 64, 1942) and Tyreus-Luyben (Ind. Eng. Chem. Res. 31(11), 1992). SIMC (Skogestad, J. Process Control 13, 2003) is not used: it needs a step-response model, and the heave loop is open-loop unstable.
   - LQR: Bryson's rule (Bryson and Ho, Applied Optimal Control, 1975), then iterative re-weighting that tightens whatever overshot its target. This loop is the project's own heuristic, not a published method.
3. a cross-entropy search (Rubinstein, Methodology and Computing in Applied Probability 1, 1999) seeded at the best of 1-2.

CLI: `python -m hydrofoil.tuning.autotune --preset manned --target auto|pid|lqr`. `--target auto` (default) runs a two-stage phase-aware search: Stage 1 tunes the PID gains on the Rising (`takeoff`, `supervised`) and Falling (`landing`) scenarios where PID owns control, and Stage 2 locks those PID gains and tunes the LQR weights (with `iphi`/`itheta` coupled to `phi`/`theta`) across Cruising & 3D Turning (`head`, `following`, `shift`, `noisy`, `turn`) and full-envelope handovers. The dashboard's Autotuner tab runs the same thing.

Manned results (275 kg; height rms, mm):

| | Head sea | Following sea | Rider moves (peak) | Head sea, noisy sensors | Drive limited, noisy |
|---|---|---|---|---|---|
| PID, model gains | 26 | 95 | 183 | 55 | 11 % |
| PID, tuned (applied) | 27 | 64 | 113 | 52 | 8 % |
| LQR, default weights | 27 | 36 | 23 | 27 | 0 % |
| LQR, tuned (applied) | 18 | 18 | 10 | 25 | 1 % |

Lessons from the runs:
- **Perfect sensors mislead.** The first PID tune (four scenarios, ideal sensors) reached kp_heave 1165 deg/m and cut head-sea error to 9 mm, but on the noisy sensors it held the drives at their torque limit 52-63 % of the time. Adding the noisy-sensor scenario, and charging drive limiting steeply in the cost, removed it.
- **Classical rules overshoot here.** With ideal sensors Ziegler-Nichols from the relay test was the best single candidate (cost 8.3 against 25.8). With noisy sensors and drive effort counted, every rule set scored worse than the existing gains (Z-N 76, Tyreus-Luyben 40, current 34). The heave relay locks onto the fast flap-drive dynamics (Ku 930 deg/m, Pu 0.47 s), so the rules give gains far too high for a noisy height signal.
- **The PID cannot match the LQR.** It is three single-input loops on a coupled plant, so the tuned PID mainly speeds up takeoff (settle 15.1 to 13.8 s) and absorbs rider shifts. Fly the LQR; keep the PID for takeoff.
- **The tuned LQR costs delay margin:** 80 to 60 ms at 8-9.5 m/s, 120 to 80 ms at 5.5 m/s. That is still six times the 10 ms loop, but it is the price of the tighter height weights.

Found while checking the tuned gains against the supervisor (its tests failed: emergency landing during takeoff):
- **LQR handover kick:** at engagement the boat was still 0.11 m low. The tighter height weights turned that into a one-step jump of the commands, from +6 to +20 deg (left flap) and from -1 to -14.5 deg (elevator), which the clamped integrators could not absorb. `LQRController` now blends from the PID's command to its own over `blend_s` = 0.5 s.
- **False "stuck" in the supervisor:** a drive counted as jammed if its encoder had not moved over the last 0.2 s while the error was large, which is also true of a healthy drive in the step right after a command jump. Stuck now also needs the error to have lasted that whole window. The supervisor's fault tests still pass.
- **The autotuner's scenarios do not run the supervisor.** A tune can still trip a supervisor check this way; run the supervisor tests after applying tuned gains.

### Flight observer (`hydrofoil/monitor.py`)
An analysis tool: it reads the true state and wave field and grades each check ok / caution / danger with a plain reason. The checks are:
- hull clearance to the local surface;
- main-foil depth in chords (breach and ventilation);
- heel and pitch against the supervisor's limits;
- a flap within 1 deg of its command limit;
- drives limited over the last second;
- speed under the foiling minimum;
- growing height or heel oscillation (detrended rms over the last 3 s against the 3 s before);
- rider vertical acceleration (rms and slam peaks);
- cavitation number;
- height error.

The in-flight checks start once the boat has been foilborne for 2 s and has come within 15 % of the target height, so the takeoff climb does not trigger them. In calm cruise it stays quiet. In a 35 cm following sea it flags slams and growing oscillation. With the elevator drive cut at 15 s it gives a pitch caution at 15.9 s and a danger at 16.1 s. It is shown in the 3D view's instrument strip (worst alert) and on the dashboard: an alert list, a log, and shading on the traces.

### Boat editor (`hydrofoil/design.py`, dashboard "Boat editor" tab)
`apply_design` turns edits into a consistent parameter set:
- **Payloads** (point masses replacing the preset's `default_payloads`, which is a 75 kg rider on the manned preset) set the total mass, the CG offset (x, y) and the inertia, by the parallel-axis theorem about the new CG. Vertical CG is reported but not flown.
- **Strut length** moves both foils.
- **Main half-span** keeps the 0.1 m root gap.

When an edit changes trim, the trim feed-forward and the LQR / estimator speed schedule are re-derived from the speeds where the boat can trim with the flap inside +-15 deg.

`derived` reports the following without flying, from the trim curve, and the dashboard shows them against the preset:
- wing area, aspect ratio and wing loading;
- the slowest trimmable speed;
- cruise flap and its margin;
- cruise thrust and power;
- foil depth;
- cavitation number and Reynolds number.

Trim is computed at the actual CG. The trim curve is cached on the physical parameters only. Foil shape is limited to what the model has: rectangular planform and one section, NACA 0009.

Example: main chord 0.28 to 0.24 m raises wing loading 4380 to 5110 N/m2 and the slowest trim speed 5.0 to 5.5 m/s, and lowers cruise power 1.52 to 1.42 kW.

### 3D view
- The struts are drawn at their real chord with foil pods.
- The main-foil halves are joined across the root.
- A bracket carries the elevator strut, which sits 0.2 m aft of the transom on the manned preset (a real feature of these parameters: elevator 2.0 m aft, hull ends at 1.8 m).
- Planing hull mesh and a seated rider on the manned preset.
- Instrument strip in the dashboard's type and colours.
- D opens the dashboard; the dashboard's "Show 3D view" brings the window forward.
