# 04 — Constants and Parameters

"Provenance" column: **guess** = round-number engineering guess, **placeholder** = explicitly labelled as such in code,
**tuned** = found by sweep/feel, **derived** = from a formula, **physical** = true constant/standard.
There are **no measured values** anywhere in the repo.

## A. `DEFAULT_PARAMS` (`hydrofoil/dashboard.py`) — master list

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

### Hull
| Key | Default | Unit | UI range |
|---|---|---|---|
| `waterplane_area` | 0.6 | m² | 0.05–3 |
| `max_draft` | 0.3 | m | 0.05–1 |
| `hull_offset` | 0.6 | m (hull bottom above CoG) | 0–1.5 |
| `hull_drag_coeff` | 20.0 | N/(m/s)² | 0–100 (placeholder) |

### Flight targets
| Key | Default | Unit | Notes |
|---|---|---|---|
| `flying_height` | 0.55 | m | Hull bottom above water when foilborne. `z_ref = hull_offset − flying_height` (=0.05 m CoG depth). Range −0.3…1.0 |
| `phi_ref` | 0.0 | deg | Roll target; dashboard UI ±10°, UDP uplink ±8° |
| `theta_ref` | 0.0 | deg | Pitch target ±10°; **only applied on restart** |
| `target_speed` | 4.0 | m/s | 0–15 in UI (uplink 0–10) |

### Gains (see doc 06)
| Loop | Kp | Ki | Kd | Note |
|---|---|---|---|---|
| heave | 15.0 | 1.0 | 5.0 | higher than `sim.py` for takeoff authority |
| pitch | 2.0 | 0.0 | 2.0 | |
| roll | 3.5 | **0.0 (must stay 0)** | 2.5 | swept; Ki>0 diverges |
| speed | 30.0 | 5.0 | 0.0 | → thrust [N] |

`max_thrust` = 50 N (placeholder matching Unity arcade-mode value).

### Waves
`waves_enabled` False; `wave_amplitude` 0.05 m; `wave_wavelength` 5.0 m; `wave_period` 2.0 s; direction fixed 0 (+x) in dashboard.

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
| `DEFAULT_ACTUATOR_PARAMS` | rate 720°/s, τ 0.02 s, delay 2 steps | `control/actuator.py` — placeholder for MyActuator RMD-X8-class |

At dt = 0.01 the delay is 20 ms. The delay is in *steps*, so changing dt changes the real delay.

## D. Safety constants (`vehicle.py`)
`MAX_ATTITUDE_RAD` 70°, `MAX_LINEAR_VEL` 15 m/s, `MAX_ANGULAR_VEL` 3 rad/s.

## E. Pilot-uplink constants (`control_uplink.py`)
Height −0.3…1.0 m; speed 0…10 m/s; bank ±8°. UDP in: 5006. UDP out: 5005.

## F. Physical constants
ρ_water 1025 kg/m³ (hard-coded in several places: `foil.py`, `hull.py`, `airy.py`, `vehicle.rest_state`, `rigid_body.py`), g 9.81 m/s².

## G. `sim.py` legacy constants
mass 25, inertia diag(2,5,5), DISPLACED_VOLUME = 25/1025, main half 0.15×0.6, elevator 0.10×0.3, MAIN_FOIL_POS (0,0,0.5),
ELEV_FOIL_POS (−1.2,0,0.4), lateral offset 0.35, gains heave (1,0,3) / pitch (2,0,2) / roll (3.5,0,2.5), DT 0.01, T_END 20, CRUISE 4 m/s,
Z_REF 0.5, heave damping 50 (hard-coded), start z 0.3 with 2° pitch.

## H. Unity-side constants (`PythonSimDriver.cs`)
HEIGHT_RATE 0.15 m/s, HEIGHT −0.3…1.0; SPEED_RATE 1.0 m/s², SPEED 0…10; BANK_RATE 6°/s, ±8°, return 10°/s;
send interval 0.05 s; `modelYawOffsetDeg` −90; ports 5005/5006; default sim command contains a
machine-specific path (`/home/jleathem/code/unity`) and distro `Ubuntu`.

## Likely-to-change parameters (priority for real measurement)
mass, inertia, foil chord/span/positions, hull waterplane area / drag, flap effectiveness, actuator rate/lag/delay,
max thrust, and the water density if testing on a lake.
