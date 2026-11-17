# 21 — Per-phase controller trials (`tools/phase_trials.py`)

Full sweep of controller assignment across the four supervisor phases (`DISPLACEMENT`, `TAKEOFF`, `FOILING`, `LANDING`) on `MANNED_PARAMS` with noisy sensors, Kalman/HeaveFilter estimation, roll added inertia, hull Froude drag hump, >15° plain-flap separation, breach ventilation hysteresis, propulsion spool lag, dual height redundancy, and GNSS vertical-velocity fallback.

```bash
python tools/phase_trials.py
```

## 1. Test matrix

- **36 controller combinations** (`3 × 3 × 2 × 2`):
  - `DISPLACEMENT`: `fixed` (surfaces held at `0°`), `pid` (elevator & differential flaps damp pitch/roll while `delta_main = 0°`), `lqr`
  - `TAKEOFF`: `pid`, `auto` (`pid` on hull, blended LQR handover once airborne & settled), `lqr`
  - `FOILING`: `pid`, `lqr`
  - `LANDING`: `pid`, `lqr`
- **3 sea states** (`108` total 115 s missions, `sensors="noisy"`): `calm`, `head` (`0.15 m` amplitude, `10.8 m` wavelength, `180°`), and `following` (`0°`).
- **Mission profile:** 10 s displacement at `3 m/s` → foil takeoff request at `7.0 m/s` → cruise acceleration to `8.5 m/s` at `45 s` → deceleration to `6.5 m/s` at `65 s` → land request at `85 s` → 30 s displacement finish.

## 2. Overall pass rate (92 / 108 passed)

All **16 failures** occurred in **head seas** (`takeoff_timeout = 25 s` exceeded at `35.0 s`) and followed two exact rules:
1. **`TAKEOFF = lqr` (`12 / 12` head-sea failures):** Forcing foilborne LQR while hull-wetted in head waves fights the hull drag hump and stalls acceleration past the `25 s` timeout before retry.
2. **`DISPLACEMENT = lqr` paired with `TAKEOFF = auto` (`4 / 4` head-sea failures):** Engaging foilborne LQR in wave-tossed displacement defers/disturbs the low-speed hull trim at takeoff handover, stretching head-sea takeoff past the `25 s` timeout window.
3. Every combination with `DISPLACEMENT ∈ {fixed, pid}` and `TAKEOFF ∈ {pid, auto}` passed **48 / 48 runs (`100%`)** across calm, head, and following seas with zero faults and `0.0` limited-fraction drive saturation in cruise.

## 3. Phase-by-phase metrics (canonical comparisons)

| Combo `(D, T, F, L)` | Sea | Takeoff [s] (touches) | Takeoff [Wh] | Foiling `z` rms [mm] | Min clear [m] | Foiling `az` rms [g] | Cruise [Wh/km] | Land sink [m/s] | Post-land max pitch [°] | Total [Wh] |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `pid, pid, lqr, lqr` | calm | 9.17 (0) | 5.35 | 14.9 | 0.210 | 0.021 | 52.87 | 0.101 | 0.42 | 40.35 |
| `pid, auto, lqr, lqr` | calm | 8.54 (0) | 5.02 | 15.4 | 0.210 | 0.021 | 53.08 | 0.101 | 0.42 | 40.37 |
| `pid, auto, pid, lqr` | calm | 8.54 (0) | 5.02 | 16.4 | 0.208 | 0.018 | 52.98 | 0.095 | 0.52 | 40.34 |
| `fixed, auto, lqr, lqr` | calm | 8.54 (0) | 5.02 | 15.4 | 0.210 | 0.021 | 53.08 | 0.136 | 0.58 | 40.38 |
| `pid, pid, lqr, lqr` | head | 20.40 (2) | 14.83 | 15.6 | 0.086 | 0.031 | 54.03 | 0.054 | 4.56 | 45.09 |
| `pid, auto, lqr, lqr` | head | **12.22 (1)** | **10.99** | **14.7** | **0.126** | **0.031** | **52.86** | **0.054** | **4.56** | **44.02** |
| `pid, auto, pid, lqr` | head | 12.22 (1) | 10.99 | 41.7 | 0.013 | 0.084 | 53.11 | 0.024 | 4.56 | 44.18 |
| `fixed, auto, lqr, lqr` | head | 12.21 (1) | 10.93 | 14.7 | 0.126 | 0.031 | 52.87 | 0.053 | 5.38 | 43.36 |
| `pid, pid, lqr, lqr` | following | 18.16 (1) | 16.82 | 48.8 | 0.082 | 0.029 | 53.07 | 0.133 | 4.03 | 49.76 |
| `pid, auto, lqr, lqr` | following | **14.53 (0)** | **14.65** | **49.0** | **0.057** | **0.029** | **52.88** | **0.133** | **4.03** | **48.96** |
| `pid, auto, pid, lqr` | following | 14.53 (0) | 14.65 | 79.7 | −0.027 | 0.032 | 54.03 | 0.135 | 4.02 | 49.45 |
| `fixed, auto, lqr, lqr` | following | 14.59 (0) | 14.77 | 49.0 | 0.060 | 0.029 | 52.85 | 0.130 | **18.07** | 48.96 |

## 4. Recommended baseline (`pid, auto, lqr, lqr`)

1. **`DISPLACEMENT` = `pid`:** Keeps elevator pitch and differential-flap roll damping active in waves without pulling main-wing lift. Leaving surfaces `fixed` at `0°` saves `~0.6 Wh` in head waves before takeoff, but after `LQR` touchdown in following seas (`fixed, auto, lqr, lqr`) dropping suddenly to un-damped `0°` surfaces allowed post-landing wave pitch to spike to **18.07°** (`2.12 m/s` vertical velocity) vs **4.03°** (`0.105 m/s`) under `pid`.
2. **`TAKEOFF` = `auto`:** Cuts head-sea takeoff duration by **40%** (`20.40 → 12.22 s`) and reduces takeoff energy (`14.83 → 10.99 Wh`) and hull slap count (`2 → 1`) compared to keeping PID throughout takeoff, while avoiding the `12 / 12` head-sea takeoff aborts of forcing `lqr` early.
3. **`FOILING` = `lqr`:** Cuts head-sea height error `2.8×` (`41.7 → 14.7 mm` rms) and ride acceleration `2.7×` (`0.084 → 0.031 g`), increases head-sea clearance nearly `10×` (`0.013 → 0.126 m`), and avoids hull wave contacts in following seas (`+0.057 m` min clearance, `0` contacts vs PID's `−0.027 m`, `2` contacts). Peak head-sea drive current stays `19.9 A` with `0.0` limit-fraction saturation.
4. **`LANDING` = `lqr`:** Maintains tight pitch and roll attitude down to gentle hull touchdown (`0.054–0.133 m/s` sink) before handing directly into damped `pid` displacement control.
