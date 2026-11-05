# 04 — Constants and Parameters

"Provenance" column: **guess** = round-number engineering guess, **placeholder** = explicitly labelled as such in code,
**tuned** = found by sweep/feel, **derived** = from a formula, **physical** = true constant/standard.
There are **no measured values** anywhere in the repo.

## A. `DEFAULT_PARAMS` (`hydrofoil/params.py`) — master list

### Vehicle / rigid body
| Key | Default | Unit | UI range | Provenance |
|---|---|---|---|---|
| `mass` | 25.0 | kg | 1–100 | guess |
| `ixx`, `iyy`, `izz` | 2.0, 5.0, 5.0 | kg·m² | 0.1–20 | guess (no products of inertia) |
| `heave_damping` | 50.0 | N·s/m | 0–200 | tuned by feel |

### Foil geometry (body frame: x fwd, y stbd, z down, from CoG)
| Key | Default | Unit | UI range |
|---|---|---|---|
| `main_chord` / `main_span` | 0.15 / 0.6 | m | 0.05–0.5 / 0.1–2 (span is **per half**) |
| `main_pos_x` / `main_pos_z` | 0.0 / 0.5 | m | −2…2 / 0.1–1.5 |
| `main_lateral_offset` | 0.35 | m | 0.05–1.0 (halves at y = ±offset) |
| `elev_chord` / `elev_span` | 0.10 / 0.3 | m | 0.05–0.4 / 0.1–1.5 |
| `elev_pos_x` / `elev_pos_z` | −1.2 / 0.4 | m | −3…0 / 0.1–1.5 |
| `main_strut_chord` / `elev_strut_chord` | 0.12 / 0.08 | m | **new placeholders** (doc 14 E10); wetted length is computed from depth |

### Hull
| Key | Default | Unit | UI range |
|---|---|---|---|
| `hull_length` / `hull_beam` | 2.0 / 0.30 | m | 0.5–4 / 0.1–1.5 (placeholders; waterplane area = L × B = 0.6 m², formerly `waterplane_area`) |
| `hull_x` | 0.0 | m | −1…1 (footprint centre relative to the CoG; 0 = floats level) |
| `max_draft` | 0.3 | m | 0.05–1 |
| `hull_offset` | 0.6 | m (hull bottom above CoG) | 0–1.5 |
| `hull_drag_coeff` | 20.0 | N/(m/s)² | 0–100 (placeholder) |
| `hull_rot_damping` | 400 | N·m·s/rad at full draft | placeholder radiation damping on roll/pitch, scaled by wetted fraction |

### Flight targets
| Key | Default | Unit | Notes |
|---|---|---|---|
| `flying_height` | 0.55 | m | Hull bottom above water when foilborne. `z_ref = hull_offset − flying_height` (=0.05 m CoG depth). Range −0.3…1.0 |
| `phi_ref` | 0.0 | deg | Roll target; dashboard UI ±10°, UDP uplink ±8° |
| `theta_ref` | 0.0 | deg | Pitch target ±10°; **only applied on restart** |
| `target_speed` | 4.0 | m/s | 0–15 in UI (uplink 0–10) |

### Gains (see doc 06) — retuned 2026-11-04 (doc 14 E1/E2)

> **Units:** heave °/m; roll and pitch **° of flap per ° of error** (the controller now converts radians to degrees; previously these were °/rad, 57× weaker); speed N per m/s.

| Loop | Kp | Ki | Kd | Old values (for reference) |
|---|---|---|---|---|
| heave | 40.0 | 6.0 | 10.0 | 15 / 1 / 5 |
| pitch | 1.0 | 0.0 | 0.1 | 2 / 0 / 2 |
| roll | 2.0 | 0.0 (keep 0) | 0.2 | 3.5 / 0 / 2.5 |
| speed | 30.0 | 5.0 | 0.0 | unchanged |

`max_thrust` = 50 N (placeholder). Measured cruise thrust at 4 m/s is ≈ 14 N.

### Waves
`waves_enabled` False; `wave_amplitude` 0.05 m; `wave_wavelength` 5.0 m; `wave_direction` 0° (travel heading, −180…180; 90 = beam seas). **`wave_period` was removed**: the period now follows deep-water dispersion (T = 2π/√(g k) = 1.79 s for 5 m).

### Simulation
`dt` 0.01 s (0.001–0.05); `t_end` 60 s (5–120).

## B. Foil-model constants (`forces/foil.py`)
| Name | Value | Meaning |
|---|---|---|
| `MAIN_FOIL_FLAP_EFFECTIVENESS` | 0.21 | dα_eff/dδ — plain flap, 40 % chord, inboard half-span |
| `ELEVATOR_FLAP_EFFECTIVENESS` | 1.0 | all-moving elevator |
| `MAIN_FOIL_OSWALD_EFFICIENCY` | 0.80 | e in CL²/(π·AR·e); split wing |
| `Foil.rho` | 1025 kg/m³ | seawater (note: lake/fresh water ≈ 998–1000) |
| `Foil.lift_axis` | (0,0,−1) | up |
| speed cut-off | 0.01 m/s | below → zero force |
| `DECAY_LENGTH_CHORDS` | 1.58 | free-surface fit: factor(3)≈0.85, factor(10)≈0.998 (placeholder) |
| AR correction | AR/(AR+2) | finite-wing limit; 0.667 at AR 4 |

Polar: NACA 0009, XFLR5 v6.61, Re 2.7e6, Ncrit 9, α −10°…+17.8°, CL_max ≈ 1.42 @ 14.4°, CL slope ≈ 0.108/deg (test asserts 0.10–0.12).

## C. Control constants
| Name | Value | File |
|---|---|---|
| Flap command limit | ±20° (all three PIDs) | `control/pid.py` (`lim`) |
| Speed PID limit | ±`max_thrust` | `control/surge.py` |
| `DEFAULT_ACTUATOR_PARAMS` | rate 720°/s, τ 0.02 s, **delay 0.02 s** | `control/actuator.py` — placeholder for MyActuator RMD-X8-class |

The delay is now specified in **seconds** and converted to whole steps (`round(delay_s/dt)`), so changing dt no longer changes the latency.

## D. Safety constants (`vehicle.py`)
`MAX_ATTITUDE_RAD` 70°, `MAX_LINEAR_VEL` 15 m/s, `MAX_ANGULAR_VEL` 3 rad/s.

## E. Pilot-uplink constants (`control_uplink.py`)
Height −0.3…1.0 m; speed 0…10 m/s; bank ±8°. UDP in: 5006. UDP out: 5005.

## F. Physical constants
ρ_water 1025 kg/m³ and g 9.81 m/s² now live only in `hydrofoil/constants.py` (`RHO_WATER`, `G`) and are imported everywhere.

## G. `sim.py` (now a wrapper over `VehiclePhysics`)
No private constants any more: it uses `DEFAULT_PARAMS`, runs from rest, and `T_END = 90 s` (was 20 s).

## H. (Removed) Unity-side constants (`PythonSimDriver.cs`, history only)
HEIGHT_RATE 0.15 m/s, HEIGHT −0.3…1.0; SPEED_RATE 1.0 m/s², SPEED 0…10; BANK_RATE 6°/s, ±8°, return 10°/s;
send interval 0.05 s; `modelYawOffsetDeg` −90; ports 5005/5006; default sim command contains a
machine-specific path (`/home/jleathem/code/unity`) and distro `Ubuntu`.

## Likely-to-change parameters (priority for real measurement)
mass, inertia, foil chord/span/positions, hull waterplane area / drag, flap effectiveness, actuator rate/lag/delay,
max thrust, and the water density if testing on a lake.

## I. Measured/derived values from the audit (calm water, defaults, 4 m/s, run 2026-11-04)
| Quantity | Value |
|---|---|
| Added mass heave / pitch | 24.15 kg / 3.48 kg·m² |
| Rest draft | 0.041 m |
| Main-flap lift slope | 10.2 N/deg per half (both halves 20.4 N/deg) |
| Foil heave stiffness dL/dw | 698 N/(m/s) per half (explicit `heave_damping` = 50 is ~28× smaller) |
| Trim flap (cruise) | ≈ 14.1° (≈ 72 % of the ±20° limit) |
| Cruise thrust | ≈ 13–14 N |
| Minimum foilborne speed (flap-limited) | ≈ 3.1 m/s |
| Settled pitch | −0.38° |
| Open-loop eigenvalues | −82.2, −47.2, −22.7, −0.32±0.47j, −0.13±0.98j, −0.025, **+0.178**, 0 ×3 |
| cond(M) | 24.6 |
