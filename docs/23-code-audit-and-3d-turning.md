# 23 — Three-Tier Code & Logic Audit and 3D Split-Front-Flap Turning Analysis

This document records the full bottom-up adversarial audit of the repository (individual modules → coupled subsystems → whole closed-loop vehicle) and the transition from a primarily 2D vertical-plane flight model to full 3D lateral-directional turning physics and control for the actual physical control layout:

- **Front wing (`main`)**: two split flaps down the centerline (`left`, `right`) controlling **heave** (collective $\delta_{\text{main}}$) and **roll** (differential $\delta_{\text{roll}}$), where $\delta_L = \delta_{\text{main}} - \delta_{\text{roll}}$ and $\delta_R = \delta_{\text{main}} + \delta_{\text{roll}}$.
- **Rear wing (`elev`)**: one continuous all-moving elevator controlling **pitch only** (zero roll or yaw actuation from the back).
- **Yaw actuation**: **none** (no rudder). Heading changes occur solely by banking the craft with the split front flaps and letting tilted lift plus strut hydrodynamics turn the vehicle.

All assumptions, placeholder parameters, and derivations referenced here are indexed separately in [22-assumptions-and-provenance.md](22-assumptions-and-provenance.md).

---

## 1. Executive Verdict: "Do Things Make Sense?"

**Yes — the core 6-DOF Fossen architecture, linearisation, delay-aware LQR, compliant servo drive model, dual-height estimation, and flight-mode supervisor are physically sound and internally consistent — with three important caveats uncovered and addressed during this audit:**

1. **Real edge-case bugs existed in individual modules and tools** (13 distinct bugs found and fixed in §2): emerged foils still produced up to `-892 N` phantom water drag and `-1817 N·m` pitch moment in air; Airy wave orbital velocity blew up exponentially (`12.3×`) above still water; single-speed LQR/Kalman tables produced `0/0 = NaN`; sensors running faster than `1/dt` drifted behind simulation time; `design.py` produced invalid inertias when total mass dropped below `default_payloads`; `viz.py` drew pitch upside-down; and `sim_server.py` had a Windows path-traversal check bug and unclosed log handles.
2. **Lateral-directional 3D turning physics was previously incomplete for steep banked turns:** the vehicle lacked strut sway/yaw added mass and the destabilizing yaw Munk moment, lacked displacement hull sway drag and yaw rotational damping, checked free-surface lift loss and breach ventilation only at the **panel center** (`y = 0.60 m`) rather than the **outboard wingtip** (`y = 1.15 m`, missing full tip breach at `35°` bank), lacked strut sideslip ventilation, and had no steady-turn equilibrium feedforward, split-flap saturation allocator, or dynamic wave/speed bank limiter.
3. **One historical `MANNED_PARAMS` baseline caveat (addressed in the autotuner):** the earlier `MANNED_PARAMS` LQR weights (prior to the phase-aware autotuner update) relaxed the roll angle weight `phi` from `0.8°` to `4.52°` without relaxing the roll-integral weight `iphi` (`0.8°·s`). Consequently the baseline integral roll gain `K_roll[iphi] = -256.1` exceeds the proportional roll gain `K_roll[phi] = -153.7` by `1.67×`, causing a `~1.9–2.4°` transient bank overshoot on bank maneuvers (`8° → 9.9°`, `15° → 17.3°`). We kept the baseline `MANNED_PARAMS` weights unchanged in `params.py`, while `hydrofoil/tuning/autotune.py` now couples `iphi`/`itheta` proportionally to `phi`/`theta` and includes the `turn` and `landing` scenarios inside the default `--target auto` phase-aware workflow (§6.4).

---

## 2. Three-Tier Adversarial Audit Results

### 2.1 Tier 1 — Individual Module Break Tests & Fixes

Every standalone physics, actuation, sensing, control, design, and UI module was stress-tested at boundary inputs, extreme states, degenerate configs, and passivity/symmetry invariants.

| # | Module & Location | Adversarial Break Test | Pre-Fix Failure Mode | Fix Implemented | Post-Fix Verification |
|---|---|---|---|---|---|
| **A1** | `hydrofoil/forces/foil.py` (`generalized_force`) | Foil emerged above water (`h/c = -0.5, vent = 1.0`, `u = 8 m/s`, `delta = 10°`) | Returned `Fx = -891.9 N`, `My = -1817.4 N·m`, `Nz = -535.2 N·m` phantom water drag and pitching moment in air (`CD_2d`, cavity drag, and `flap_cm_delta` never went to zero when `h/c <= 0`) | Early return `np.zeros(6)` with `last_CL = 0.0` when `h/c <= 0`; smoothstep emergence wetting over `0 < h/c < 0.25` scaling `CD_2d`, cavity drag, and `flap_cm_delta`; `flap_cm_delta` also attenuated by `(1 - 0.55 * vent)` | `tau == [0,0,0,0,0,0]` (`test_emerged_foil_has_zero_phantom_drag_and_pitching_moment`) |
| **A2** | `hydrofoil/waves/airy.py` (`AiryWave`) | Evaluate `orbital_velocity` above SWL (`z = +2.0 m`, `A = 0.25 m`, `λ = 5 m`) and pass invalid `wavelength <= 0` / `period <= 0` | `exp(k * z)` exploded `|V|` from `0.878 m/s` at `z = 0` to `10.836 m/s` (`12.3×`) at `z = +2 m`; non-positive wavelength caused silent division-by-zero / negative wavenumber | Clamped `z_eff = min(float(z), 0.0)` in `orbital_velocity`; added `ValueError` validation on `wavelength <= 0` and `period <= 0` | `|V(+2 m)| == |V(0)| = 0.878 m/s`; invalid inputs raise `ValueError` |
| **A3** | `hydrofoil/dynamics/rigid_body.py` + `vehicle.py` | Check `M[1,1]`, `M[5,5]`, yaw Munk moment `-(C_A nu)[5]`, and skew-symmetry passivity `nu^T C(nu) nu == 0` | Zero sway/yaw added mass (`M[1,1] = 275 kg`, `M[5,5] = 280 kg·m²`) and zero yaw Munk moment in sideslip | Added `strip_added_mass_sway_yaw` (`27.82 kg` / `37.10 kg·m²` on manned; `8.26 kg` / `3.70 kg·m²` on model) into `M` and `C_A` | Exact yaw Munk `-111.3 N·m` at `u = 8, v = 0.5`; passivity `|nu^T C nu| < 1e-12` |
| **A4** | `hydrofoil/forces/hull.py` (`HullHydrostatics.drag`) | Displacement lateral motion `v = 2 m/s, r = 0.5 rad/s, draft = 0.15 m` | Returned `Fy = 0.0 N`, `Nz = 0.0 N·m` (unbounded displacement drift and yaw spinning) | Added crossflow sway drag `Fy = -0.5 * rho * L * d_max * wetted * v|v|` and yaw rotational damping `Nz = -rot_damping * wetted * r` (both zero when foilborne) | `Fy = -1107.0 N`, `Nz = -1500.0 N·m` in water; strictly `0` when `draft = 0` |
| **A5** | `hydrofoil/sensors/models.py` (`Channel.step`) | Run `gyro` (`200 Hz`, period `5 ms`) inside a `dt = 0.01 s` (`100 Hz`) loop for `10.0 s` | `self._next += period` fell steadily behind (`_next = 5.00 s` at `t = 10.0 s`) | Advance `while self._next <= t + 1e-12: self._next += self.period` | `_next = 9.995 s` after `10.0 s` |
| **A6** | `hydrofoil/estimation/kalman.py` + `control/lqr.py` | Construct `LQRController` or `KalmanEstimator` with a 1-element schedule `speeds=(8.0,)` | `searchsorted` clipped to `j = 0`, `k = 0`, dividing `(u - s[0]) / (s[0] - s[0]) = 0/0 = NaN` | Added explicit `len(self.speeds) == 1` guard returning `(0, 0, 0.0)` / `arr[0]` | Finite `K` and `Ad` with single-speed schedules |
| **A7** | `hydrofoil/forces/buoyancy.py` (`buoyancy_wrench`) | Profile inner RK4 loop overhead (`docs/11` C9) | `from hydrofoil.dynamics.kinematics import R_body_to_ned` executed on every call (4× per RK4 step) | Hoisted import to module top level | Eliminates per-stage import overhead |
| **A8** | `hydrofoil/control/pid.py` | Inspect sign conventions on roll PID (`L106–110`, `L186`) | Comments claimed opposite sign (`delta_roll > 0` rolls right) from the actual physics (`delta_roll > 0` rolls **left**, `delta_L = main - roll`, `delta_R = main + roll`) | Corrected comments to match exact code & Fossen sign convention | Verified against `test_split_front_flaps_produce_roll_and_adverse_yaw_while_back_wing_is_pure_pitch` |
| **A9** | `hydrofoil/design.py` (`apply_design`, `mass_properties`) | Edit manned preset (`default_payloads = 75 kg` rider) to `mass = 50 kg` | Subtracting rider mass produced negative bare-hull mass and bogus `ixx = 2.04 kg·m²` | `mass_properties` raises `ValueError` on `bare_mass <= 0`; `_scale_inertias_and_hull` falls back to total-mass scaling when `mass <= sum(payloads)` | Clean `ixx = 14.55 kg·m²` fallback and explicit `ValueError` guard |
| **A10** | `hydrofoil/sim_server.py` | Request `/fonts/..\secret` on Windows; inspect spawned subprocess log FDs | Prefix check missed Windows backslash separator normalization; `open(LOG_PATH, "ab")` leaked open file handles on repeated launches | Replaced with `resolve().is_relative_to(FONTS.resolve())` and `with open(...)` context manager | Traversal rejected cleanly; log handle closed immediately after `Popen` |
| **A11** | `hydrofoil/viz.py` (`animation_2d`) | Plot craft pitched nose-up `+10°` (`theta = +0.1745 rad`) | Rotation mapped bow `(+1, 0)` to `z = +0.1736` (DOWN) instead of `-0.1736` (UP), and y-axis wasn't inverted for NED depth | Fixed `body_to_world` to `(bx*c + bz*s, -bx*s + bz*c + zi)` and inverted y-axis range `[water_depth, -0.4]` | Nose-up `+10°` renders bow up above waterline |
| **A12** | `hydrofoil/vehicle.py` (`_servo_commands`, kinematic mode) | Run `actuator_model="kinematic"` with `sensors="noisy"` and `supervisor=True`; check `q_schedule_speed` | Legacy kinematic branch bypassed noisy sensor state `ceta` and skipped `supervisor.shape_raw`; `q_schedule_speed` leaked true `state[6]` instead of measured `cnu[0]` | Wired kinematic branch through `ceta` + `supervisor.shape_raw`; switched `q_schedule_speed` to `cnu[0]` | Zero oracle leakage in servo `q`-schedule; consistent supervision across actuator models |
| **A13** | `hydrofoil/vehicle.py` + `monitor.py` | Bank craft `25°–35°` at `h = 0.50 m` and check free-surface lift loss & breach ventilation | Checked submergence only at panel center (`y = 0.60 m`): at `35°` bank, center was `0.85 c` (safe!) while the outboard wingtip (`y = 1.15 m`) was `-0.27 c` (**breached out of water**) | Added `_foil_tip_submergence_over_chord`, 3-station spanwise Simpson `_effective_submergence_over_chord`, tip-first breach `_vent` (partial `0.55` on tip-only breach up to `1.0` on full breach), strut sideslip ventilation (`8.5° / 4.5°`), and tip/sideslip monitor alerts | Tip breach detected cleanly; smooth outer-panel lift drop in steep banks |

### 2.2 Tier 2 — Coupled Subsystem Break Tests

| Subgroup | Stress Test Conducted | Pre-Fix Outcome | Post-Fix Outcome |
|---|---|---|---|
| **2A. Actuation Chain** (`hinge`, `drive`, `servo`, `loads`) | `30°` mechanical stop slam, `150°/s` reversal across `0.10°` backlash, `15°–20°` banked turn hinge loads, 3-drive rail power sum | Healthy; `consistency.py` passed; mechanical stops, unilateral backlash, and dual-encoder damping held stable with zero spikes | Unchanged healthy; split-flap allocator prevents over-driving flaps past `±20°` during high-bank turns |
| **2B. Hydrodynamics + 6-DOF** (`foil`, `hull`, `airy`, `rigid_body`, `vehicle`) | Bank sweep `0°..35°`, sideslip sweep `0°..15°`, reverse flow, emergence, oblique `0.25 m` waves | Emerged foil phantom drag (A1), Airy blowup above SWL (A2), missing sway/yaw added mass & Munk moment (A3), zero displacement lateral damping (A4), missed wingtip breach at `35°` bank (A13) | All fixed; passive skew-symmetric `C(nu)` verified; strut sideslip ventilation and wingtip-first breach ventilation active |
| **2C. Sensors + Estimation** (`models`, `kalman`, `HeaveFilter`) | Steady noisy banked turns `8° / 12° / 15°` in calm and `0.15 m` oblique waves; single-speed schedule; `200 Hz` gyro at `dt = 0.01 s` | `KalmanEstimator` tracked turns well (`v_err <= 0.006 m/s`, `r_err <= 0.02°/s` calm; `v_err <= 0.035 m/s` in `0.15 m` waves); broke on single-speed `NaN` (A6) and gyro `_next` drift (A5) | All fixed; confirmed linear Kalman estimator does **not** diverge in turns up to `15°` bank (note: real hardware AHRS centripetal turn-acceleration error is still unmodeled — see `docs/22 §8`) |
| **2D. Control + Supervisor** (`pid`, `lqr`, `supervisor`) | Unsupervised `u = 6 / 8 / 9.5 m/s × phi = 5°..25°` LQR & PID sweep; supervised `15°` turns in calm and `0.25 m` waves | **LQR at `u = 6, 25°` struck hull** (`dz = 262.9 mm`, `dR = +26.9°`, `i_phi = +0.030` clamped, `tip = 0.26 c`); LQR height sag was `29.7 mm` at `8 m/s, 15°`; **PID overbanked `+11.4%`** (`15° → 16.67°`, tripping old `15°` fault limit) with `67.4 mm` height drop; supervisor had **no bank clamp or bank ramp** (`8°` step spiked `10.5°` roll, S-turn spiked `beta = 7.1°`) | **18/18 LQR sweep cases flew cleanly with zero hull strikes** (even `u = 6, 25°` held `dz = 148.8 mm, dR = +17.8°, tip = 0.76 c`); height sag cut **33–40%** and roll integrator cut **3.4–5.8×**; supervisor enforces `safe_bank_limit` + `12°/s` ramp |
| **2E. Design, Consistency, Monitor, Viewer, Server** | `design.py` extreme mass/geometry edits, `consistency.py` on both presets, `sim_server.py` security & FDs, `viz.py` projection | `design.py` broke at `mass = 50 kg` (A9); `sim_server.py` backslash traversal (A10); `viz.py` inverted pitch (A11); `monitor.py` ignored wingtip depth & sideslip (A13) | All fixed; `consistency.py` passes with **0 FAILs** on both presets; viewer HUD/telemetry exports `yaw`, `yaw_rate`, `sideslip`, and live `bank_max` |

### 2.3 Tier 3 — Whole-Project Closed-Loop Mission Suite (Supervised + Noisy Sensors + LQR)

Tested end-to-end on the manned `275 kg` craft (`controller="lqr"`, `sensors="noisy"`, `supervisor=True`, compliant servo drives, `dt = 0.01 s`), starting from takeoff/cruise into demanding 3D turn missions:

| Mission | Scenario | Pre-Fix Result | Post-Fix Result |
|---|---|---|---|
| **M1** | `8°` right bank, calm (`8 m/s`) | `FOILING`, `phi_pk = 10.5°`, `beta_pk = 5.3°`, `th_pk = 0.4°` | `FOILING`, `flt=[]`, `phi_pk = 9.9°`, **`beta_pk = 3.1°`** (−42%), `th_pk = 0.2°`, `hit = 0` |
| **M1b** | `15°` right bank, calm (`8 m/s`) | Tripped old `15°` attitude fault on transient overshoot | `FOILING`, `flt=[]`, `phi_pk = 17.3°`, `beta_pk = 3.6°`, `tip_min = 1.72 c`, `clr_min = 0.48 m`, `hit = 0` |
| **M2** | S-turn `+8° → -8°`, calm (`8 m/s`) | `FOILING`, **`beta_pk = 7.1°`** (near strut ventilation!) | `FOILING`, `flt=[]`, **`beta_pk = 3.0°`** (**−58%** sideslip slam), `hit = 0` |
| **M2b** | S-turn `+15° → -15°`, calm (`8 m/s`) | Unachievable (tripped old `15°` attitude limit) | `FOILING`, `flt=[]`, `beta_pk <= 3.6°`, `th_pk <= 0.4°`, `tip_min >= 1.72 c`, `hit = 0` |
| **M3** | `15°` req in `0.20 m` beam sea | Unclamped bank risked chine strike / wingtip breach | `safe_bank_limit` auto-narrowed to `5.2°` on estimated wave; `FOILING`, `flt=[]`, `clr_min >= 0.20 m`, `hit = 0` |
| **M4** | `15°` req in `0.25 m` oblique sea | Unclamped bank risked chine strike / wingtip breach | `safe_bank_limit` auto-narrowed to `7.6°`; `FOILING`, `flt=[]`, `clr_min >= 0.20 m`, `hit = 0` |
| **M5** | `12°` bank + `+15 cm` lateral rider shift | High roll-integrator saturation risk | `FOILING`, `flt=[]`, `phi_pk = 14.2°`, `tip_min = 2.04 c`, `hit = 0` |
| **M7** | `12°` bank + **LEFT flap goes limp** at `t = 4 s` | Mid-turn asymmetry emergency | `flap_left` detected cleanly → `EMERGENCY_LANDING` → `DISPLACEMENT`, `phi_pk = 14.3°`, `th_pk <= 2.6°`, no capsize |
| **M8** | `12°` bank + **dual height sensor failure** at `t = 4 s` | Mid-turn blind altitude failure | `height_lost` detected cleanly (GNSS `vz` fallback active) → `EMERGENCY_LANDING` → `DISPLACEMENT`, controlled sink, no capsize |

---

## 3. 3D Lateral-Directional Physics of Split Front Flaps (No Rudder)

### 3.1 Control Surface Sign Conventions & Asymmetric Coupling

With two split front flaps at $y_L = -0.60\text{ m}$ and $y_R = +0.60\text{ m}$ (outboard tips at $\pm 1.15\text{ m}$) and a single-piece rear elevator at $x_e = -2.0\text{ m}, y_e = 0$:

$$
\delta_L = \delta_{\text{main}} - \delta_{\text{roll}}, \qquad \delta_R = \delta_{\text{main}} + \delta_{\text{roll}}, \qquad \delta_E = \delta_{\text{elev}}
$$

- **Positive $\delta_{\text{main}}$** deflects both front flaps downward → increases upward lift (`Fz < 0`) → craft rises (`z` decreases).
- **Positive $\delta_{\text{roll}}$** deflects the starboard (right) flap downward (`+`) and port (left) flap upward (`-`) → starboard lift rises, port lift drops → **rolls LEFT (`K < 0`, $\dot{p} < 0$)**.
- **Rolling RIGHT into a right-hand turn ($\phi > 0$)** initially requires **negative $\delta_{\text{roll}} < 0$** (left flap down, right flap up).
- **Rear elevator ($\delta_E$)** lies strictly on $y = 0$: its roll moment $K_{\delta_E} \equiv 0$, yaw moment $N_{\delta_E} \equiv 0$, and side force $Y_{\delta_E} \equiv 0$.

### 3.2 Adverse Yaw from Differential Front Flaps

Why does deflecting the split front flaps NOT produce a clean turn? Because **induced drag grows quadratically with lift** ($C_{D,i} = C_L^2 / (\pi e AR)$). When rolling RIGHT ($\delta_{\text{roll}} < 0$), the left half-wing increases its lift ($C_{L,L} > C_{L,R}$) and therefore generates **more drag** on the port side ($y_L < 0$) than the starboard side ($y_R > 0$). That drag differential produces a **nose-left yaw moment ($N < 0$) opposing the right-hand turn** — classic **adverse yaw**.

Linearized post-fix open-loop lateral derivatives (including strut sway/yaw added mass and yaw Munk moment):

| Craft & Speed | Roll Power $K_{\delta_{\text{roll}}}$ [N·m/deg] | Adverse Yaw $N_{\delta_{\text{roll}}}$ [N·m/deg] | Adverse Ratio $N / |K|$ | Weathervane $N_v$ [N·m/(m/s)] | Yaw Damping $N_r$ [N·m·s/rad] | Overbanking $K_v$ [N·m/(m/s)] | Unstable Spiral Pole [s⁻¹] |
|---|---:|---:|---:|---:|---:|---:|---:|
| **Manned 6.0 m/s** | `−148.60` | `+9.58` | **`+6.44%`** | `+2537.3` | `−5426.5` | `+4201.0` | `+0.7619` |
| **Manned 8.0 m/s** | `−236.54` | `+9.36` | **`+3.96%`** | `+3383.0` | `−7224.0` | `+5601.3` | `+0.6664` |
| **Manned 9.5 m/s** | `−338.23` | `+8.93` | **`+2.64%`** | `+4017.3` | `−8575.5` | `+6651.6` | `+0.5741` |
| **Model 4.0 m/s** | `−6.35` | `+0.29` | **`+4.60%`** | `+351.5` | `−464.3` | `+180.5` | `+0.2453` |
| **Model 6.0 m/s** | `−14.82` | `+0.28` | **`+1.87%`** | `+527.3` | `−696.0` | `+270.8` | `+0.1518` |

*Sign note:* $K_{\delta_{\text{roll}}} < 0$ (rolls left) while $N_{\delta_{\text{roll}}} > 0$ (yaws right), so $N$ and $K$ always carry **opposite signs**. Adverse yaw is worst at low speed (`6.44%` at `6.0 m/s`) because trim $C_L$ is higher, steepening $\partial C_{D,i}/\partial C_L \propto 2 C_L$.

### 3.3 Why Rudderless Turns Yield Only ~29–34% of Ideal Coordinated Turn Rate

In an aircraft with a rudder trimmed to zero sideslip ($\beta = 0$), banking by $\phi$ tilts the lift vector sideways by $L \sin\phi = m g \tan\phi$, giving the textbook ideal turn rate:

$$
\dot{\psi}_{\text{ideal}} = \frac{g \tan\phi}{u}, \qquad R_{\text{ideal}} = \frac{u^2}{g \tan\phi}
$$

At $u = 8.0\text{ m/s}$ and $\phi = 15^\circ$, ideal coordinated flight would give $\dot{\psi}_{\text{ideal}} = 18.8^\circ/\text{s}$ and $R_{\text{ideal}} = 24.3\text{ m}$. **Our rudderless hydrofoil achieves $\dot{\psi} = 5.78^\circ/\text{s}$ and $R = 77.9\text{ m}$ (~31% of ideal).** The exact physical mechanism is a 3-link causal chain:

1. **Aft strut yaw damping resists turning:** as the boat begins yawing right ($r > 0$), the aft elevator strut at $x_e = -2.0\text{ m}$ swings left through the water at lateral velocity $v_{\text{tail}} = v - 2.0 r$. Without a rudder to cancel that moment, the aft strut generates a huge restoring yaw damping torque ($N_r = -7224\text{ N·m·s/rad}$ at `8 m/s`).
2. **The boat must crab nose-in ($\beta > 0$) to balance yaw:** to overcome $N_r r$ with zero rudder, steady yaw equilibrium $N_v v + N_r r + N_{\delta_{\text{roll}}} \delta_{\text{roll}} = 0$ forces a steady positive sway velocity $v > 0$ (sliding toward the *inside* of the turn, nose pointed outside the velocity vector: $\beta \approx +1.45^\circ$ at `8 m/s, 15°`).
3. **The large front strut converts that sideslip into outward side force:** both vertical struts now experience positive sideslip $\beta > 0$ and generate strong hydrodynamic side force **outward** ($Y_v v < 0$, with $Y_v = -4378\text{ N/(m/s)}$ at `8 m/s`), cancelling **~67–70%** of the inward tilted lift $m g \tan\phi$!

Consequently, the steady rudderless turn rate is fixed by the strut geometry (`main_strut_chord = 0.16 m`, `elev_strut_chord = 0.12 m`, `elev_pos_x = -2.0 m`):
- **Manned craft:** linear steady turn rate is **28.6–29.8%** of ideal `g/u` (rising slightly to **31–34%** at finite `15–20°` bank due to geometrically tilted wetted struts and nonlinear lift).
- **25 kg Model:** only **11.6–11.8%** of ideal `g/u` because its struts have large chord relative to its `25 kg` mass.

> **Design implication for the club:** If the team wants tighter turn radii than `R ≈ 78 m` at `8 m/s, 15°` bank (or `41.5 m` at `6 m/s, 15°`), the two physical levers are: (1) adding a small trailing-edge **rudder** on the aft strut (a rudder trims $\beta \to 0$ and triples turn rate toward `R ≈ 25–30 m`), or (2) reducing `main_strut_chord` / aft strut area (trading structural stiffness and weathervane stability for tighter rudderless carving).

### 3.4 Spiral Overbanking & Steady-Turn Flap Sign Reversal

Because the struts sit **below** the CG ($z_{\text{strut}} \approx +0.9\text{ m}$), the outward side force ($Y < 0$) generated by nose-in sideslip ($v > 0$) exerts a **positive (right-rolling) torque** $K = -z_{\text{strut}} Y > 0$ ($K_v = +5601\text{ N·m/(m/s)}$ at `8 m/s`). That torque tries to roll the craft **further into the turn** — creating an open-loop unstable **spiral mode** with pole $\lambda = +0.666\text{ s}^{-1}$ at `8 m/s` (doubling time `1.04 s`) and $+0.762\text{ s}^{-1}$ at `6 m/s` (doubling time `0.91 s`).

This produces a counter-intuitive piloting and control requirement:
- **Turn entry (roll right):** deflect split front flaps **negative** ($\delta_{\text{roll}} < 0$: left down, right up) to roll the craft right.
- **Steady right turn ($\phi > 0$):** once banked and sideslipping, the split front flaps MUST **reverse sign** ($\delta_{\text{roll}} > 0$: **right flap deflected MORE downward than left flap**) to hold the low wing UP against spiral overbanking! At `8 m/s, 15°` bank, steady trim is $\delta_L = -2.52^\circ, \delta_R = +2.86^\circ$ ($\delta_{\text{roll}} = +2.69^\circ$); at `6 m/s, 20°` bank, steady trim reaches $\delta_L = +6.26^\circ, \delta_R = +18.45^\circ$ ($\delta_{\text{roll}} = +6.09^\circ$).

---

## 4. Safe Turning Degree & Envelope Analysis (Manned `275 kg` Craft)

### 4.1 Four Physical Boundaries Limiting Safe Bank Angle

Banking a T-foil craft narrows its operating corridor simultaneously from four directions:

1. **Outboard (high-side) wingtip free-surface breach & ventilation:** with half-wingtips at $y_{\text{tip}} = \pm 1.15\text{ m}$ and main foil depth $z_{\text{main}} = 1.75\text{ m}$, banking lifts the high-side tip by $y_{\text{tip}} \sin|\phi|$ while foreshortening strut vertical reach by $z_{\text{main}}(1 - \cos\phi)$. At `h = 0.50 m` calm flight:
   - `15°`: tip depth = `1.73 c` (`0.48 m`) — full lift, well clear of `0.25 c` ventilation onset.
   - `20°`: tip depth = `1.22 c` (`0.34 m`) — enters shallow free-surface lift loss.
   - `25°`: tip depth = `0.89 c` (`0.25 m`) — minimal wave margin.
   - `30°`: tip depth = `0.32 c` (`0.09 m`) — imminent breach on any ripple.
   - `35°`: tip depth = **`-0.27 c`** — **wingtip out of the water!**
2. **Inboard (low-side) hull chine wave strike:** banking lowers the low-side hull edge (`beam / 2 = 0.60 m`) toward the water by `0.60 * sin|phi|` (`+0.155 m` at `15°`, `+0.205 m` at `20°`). Combined with wave amplitude `wave_margin * A` (`1.2 * A`), raising ride height to save the chine pushes the high-side wingtip toward the surface. Eliminating ride height `h` yields an exact **geometric clearance lens** (`b_lens` in `Supervisor.safe_bank_limit`):
   $$z_{\text{main}} \cos\phi - (y_{\text{tip}} + B/2) \sin\phi \ge h_{\text{tip,min}} + h_{\text{chine,min}} + 2 \cdot m_w A_{\text{wave}} - z_{\text{hull}}$$
3. **Low-speed asymmetric flap saturation:** at `6.0 m/s`, the low (inside) flap carries **both** large collective lift (`1 / u²` plus `1 / cos(phi)` load factor) **and** large positive anti-overbanking roll deflection. At `6.0 m/s, 20°` bank, $\delta_R = +18.45^\circ$ leaves only `1.55°` below the `20°` command limit (zero wave-gust headroom). At `12°` bank, $\delta_R = +9.90^\circ$ keeps `10.1°` margin.
4. **Strut sideslip ventilation:** surface-piercing NACA 0009 struts risk ventilation above `~8.5°` local sideslip at $V > 3.5\text{ m/s}$. Steady calm turns stay below `2.7°` at `15°` bank (`6–9.5 m/s`), and the supervisor's `12°/s` bank-rate ramp keeps aggressive S-turn transient sideslip below `3.6°` (`> 2.3×` safety margin).

### 4.2 Verified Closed-Loop Turning Performance (Manned `275 kg`, Calm, `h = 0.50 m`, Post-Fix LQR)

| Speed $u$ [m/s] | Bank $\phi_{\text{cmd}}$ [deg] | Steady $\phi$ [deg] | Heading Rate $\dot\psi$ [deg/s] | Turn Radius $R$ [m] | Sideslip $\beta$ [deg] | Left Flap $\delta_L$ [deg] | Right Flap $\delta_R$ [deg] | Elevator $\delta_E$ [deg] | Min Tip Depth [$h/c$] | Peak Height Error [mm] | Safe Rating |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| **6.0** | `8.0°` | `8.00°` | `4.08°/s` | `82.6 m` | `+1.37°` | `+3.38°` | `+7.75°` | `-1.74°` | `2.38 c` | `22.9 mm` | Safe |
| **6.0** | `12.0°` | `12.00°` | `6.29°/s` | `53.5 m` | `+2.08°` | `+3.34°` | `+9.90°` | `-2.26°` | `2.06 c` | `44.0 mm` | **Max recommended @ 6 m/s** |
| **6.0** | `15.0°` | `15.00°` | `8.08°/s` | `41.5 m` | `+2.63°` | `+3.71°` | `+11.88°` | `-2.79°` | `1.79 c` | `66.9 mm` | Marginal (low gust margin) |
| **6.0** | `20.0°` | `19.98°` | `11.67°/s` | `28.2 m` | `+3.70°` | `+6.26°` | `+18.45°` | `-4.34°` | `1.29 c` | `110.4 mm` | Near flap saturation |
| **8.0** | `8.0°` | `8.00°` | `2.95°/s` | `152.8 m` | `+0.76°` | `-2.43°` | `+0.47°` | `-0.28°` | `2.39 c` | `6.5 mm` | Safe (gentle) |
| **8.0** | `12.0°` | `12.00°` | `4.52°/s` | `99.6 m` | `+1.15°` | `-2.58°` | `+1.75°` | `-0.49°` | `2.02 c` | `12.5 mm` | Safe (nominal) |
| **8.0** | `15.0°` | `15.00°` | `5.78°/s` | `77.9 m` | `+1.45°` | `-2.52°` | `+2.86°` | `-0.71°` | `1.73 c` | `18.8 mm` | **Max recommended cruise** |
| **8.0** | `20.0°` | `20.00°` | `8.11°/s` | `55.4 m` | `+1.97°` | `-1.97°` | `+5.03°` | `-1.24°` | `1.22 c` | `33.0 mm` | Fault limit (`roll_limit`) |
| **9.5** | `8.0°` | `8.00°` | `2.45°/s` | `218.3 m` | `+0.54°` | `-4.49°` | `-2.44°` | `+0.12°` | `2.38 c` | `5.4 mm` | Safe |
| **9.5** | `12.0°` | `12.00°` | `3.75°/s` | `142.7 m` | `+0.81°` | `-4.62°` | `-1.55°` | `-0.01°` | `2.01 c` | `12.3 mm` | Safe |
| **9.5** | `15.0°` | `15.00°` | `4.79°/s` | `111.7 m` | `+1.03°` | `-4.57°` | `-0.77°` | `-0.15°` | `1.72 c` | `17.6 mm` | **Max recommended @ 9.5 m/s** |
| **9.5** | `20.0°` | `20.00°` | `6.69°/s` | `79.8 m` | `+1.39°` | `-4.22°` | `+0.78°` | `-0.48°` | `1.20 c` | `28.5 mm` | Fault limit (`roll_limit`) |

### 4.3 Dynamic Safe Bank Limit Across Speed, Ride Height, and Sea State (`Supervisor.safe_bank_limit`)

The supervisor dynamically computes the maximum safe commanded bank angle every step from live speed, target ride height, and estimated wave amplitude (protecting `tip_hc_min = 1.25 c`, `chine_clear_min = 0.10 m`, and `5°` margin below `roll_limit = 20°`):

| Wave Amplitude $A$ | `u = 6.0 m/s` (`h = 0.35 / 0.50 / 0.65 m`) | `u = 7.0 m/s` (`h = 0.35 / 0.50 / 0.65 m`) | `u = 8.0–9.5 m/s` (`h = 0.35 / 0.50 / 0.65 m`) | Active Limiting Constraint |
|---|---:|---:|---:|---|
| **0.00 m (calm)** | `12.0° / 12.0° / 12.0°` | `14.0° / 14.0° / 14.0°` | `15.0° / 15.0° / 15.0°` | Speed flap-margin taper (`< 7.5 m/s`), `bank_max = 15°` above |
| **0.10 m** | `12.0° / 12.0° / 12.0°` | `14.0° / 14.0° / 12.1°` | `15.0° / 15.0° / 12.1°` | High-ride wingtip depth (`h = 0.65 m`) |
| **0.15 m** | `12.0° / 12.0° / 9.7°` | `14.0° / 14.0° / 9.7°` | `15.0° / 15.0° / 9.7°` | High-ride wingtip depth (`h = 0.65 m`) |
| **0.20 m** | `12.0° / 12.0° / 7.3°` | `13.9° / 13.2° / 7.3°` | `13.9° / 13.2° / 7.3°` | Simultaneous chine + wingtip lens (`b_lens`) & tip depth |
| **0.25 m** | `10.6° / 10.6° / 4.7°` | `10.6° / 10.6° / 4.7°` | `10.6° / 10.6° / 4.7°` | Geometric wave lens (`10.6°`) and high-ride tip breach (`4.7°`) |

---

## 5. How the Control, Estimation, Supervision & Monitoring Stack Accounts for Turning

### 5.1 Scheduled Delay-Aware LQR (`hydrofoil/control/lqr.py`)

Previously, the LQR drove `v, w, p, q, r` toward **zero** even when commanded to hold a steady bank $\phi_{\text{ref}} \ne 0$. Because a real banked turn has non-zero steady sideslip ($v > 0$), yaw rate ($r > 0$), horizontal-turn pitch rate ($q = r \tan\phi$), and level-flight body heave velocity ($w = -v \tan\phi$), the old LQR continuously fought its own steady turn — forcing the roll and heave integrators to wind up against `K_v, K_r, K_q` and causing `29.7 mm` height sag at `8 m/s, 15°` and a **hull strike** (`262.9 mm` drop, `+26.9°` clipped flap) at `6 m/s, 25°`.

Two upgrades resolve this cleanly with **zero change to straight-line flight** ($\phi_{\text{ref}} = \theta_{\text{ref}} = 0$):

1. **Plant-derived steady-turn equilibrium feedforward (`steady_turn_sensitivity` + `_turn_reference`):**
   At startup for each scheduled speed $u_i$, the controller solves the linear lateral steady-turn equations directly from the plant's own linearized `(As, Bs)` matrices:
   $$
   \begin{bmatrix} A_{vv} & A_{vr} & B_{v,\text{roll}} \\ A_{pv} & A_{pr} & B_{p,\text{roll}} \\ A_{rv} & A_{rr} & B_{r,\text{roll}} \end{bmatrix}
   \begin{bmatrix} v_{\text{ref}} / \tan\phi \\ r_{\text{ref}} / \tan\phi \\ \Delta\delta_{\text{roll}} / \tan\phi \end{bmatrix}
   =
   \begin{bmatrix} -A_{v\phi} \\ -A_{p\phi} \\ -A_{r\phi} \end{bmatrix}
   $$
   In flight at $(\phi_{\text{ref}}, \theta_{\text{ref}})$, `_turn_reference` computes the exact kinematic level-turn velocities $w_{\text{ref}} = u\tan\theta_{\text{ref}} - v_{\text{ref}}\tan\phi_{\text{ref}}$ and $q_{\text{ref}} = r_{\text{ref}}\tan\phi_{\text{ref}}$, plus the second-order body-z gravity shift $\Delta\dot{w}_{\text{grav}} = (m/M_{zz})\,g\,(\cos\phi_{\text{ref}} - 1)$ (avoiding double-counting $+(m/M_{zz}) u q$, which is already embedded in $A_{\text{lon}}$), and solves for $(\Delta\delta_{\text{main}}, \Delta\delta_{\text{elev}})$ via $B_{\text{lon}}^{-1}$.
2. **Roll-prioritized split-flap diamond allocator (`allocate_split_flaps`):**
   Because physical split flaps satisfy $\delta_L = \delta_{\text{main}} - \delta_{\text{roll}}$ and $\delta_R = \delta_{\text{main}} + \delta_{\text{roll}}$, keeping both physical flaps inside a physical ceiling $L_{\text{phys}}$ requires $|\delta_{\text{main}}| + |\delta_{\text{roll}}| \le L_{\text{phys}}$ (a diamond, not a square!). Previously, clipping $\delta_{\text{main}}$ and $\delta_{\text{roll}}$ independently to $\pm 20^\circ$ allowed commands up to $\pm 40^\circ$ on one flap, silently losing roll authority when heave saturated against the `30°` mechanical stop. `LQRController` now clips each virtual channel to `±20°` and applies `allocate_split_flaps` against `phys_limit = min(flap_stop_deg, 24°)` — reserving roll authority first (`roll = clip(roll, ±phys_limit)`), trimming collective `main` inside `±(phys_limit - |roll|)`, preserving full `±20°` wave-trough heave authority when roll noise/gusts are `<= 4°`, and applying physical-diamond saturation anti-windup to the integrators.

**Measured LQR before → after impact (manned, calm, `h = 0.50 m`):**

| Case | Height Drop `dz` (Before → After) | Roll Integrator `i_phi` (Before → After) | Peak Right Flap `dR` (Before → After) | Outcome |
|---|---:|---:|---:|---|
| `u = 6.0 m/s, 25°` | `262.9 mm → 148.8 mm` (**−43%**) | `+0.0300 (clamped) → +0.0034` (**−89%**) | `+26.9° → +17.8°` | **Hull strike → Clean flight (`tip = 0.76 c`)** |
| `u = 8.0 m/s, 8°` | `10.3 mm → 6.5 mm` (**−37%**) | `+0.0027 → +0.0008` (**−70%**) | `+0.4° → +0.5°` | Clean |
| `u = 8.0 m/s, 12°` | `20.6 mm → 12.5 mm` (**−39%**) | `+0.0039 → +0.0011` (**−72%**) | `+1.7° → +1.8°` | Clean |
| `u = 8.0 m/s, 15°` | `29.7 mm → 18.8 mm` (**−37%**) | `+0.0046 → +0.0012` (**−74%**) | `+2.8° → +2.9°` | Clean |
| `u = 8.0 m/s, 20°` | `48.9 mm → 33.0 mm` (**−33%**) | `+0.0052 → +0.0009` (**−83%**) | `+5.0° → +5.0°` | Clean |
| `u = 9.5 m/s, 25°` | `70.5 mm → 42.1 mm` (**−40%**) | `+0.0046 → +0.0005` (**−89%**) | `+1.9° → +2.2°` | Clean |

### 5.2 PID Backup Behavior in Turns (`hydrofoil/control/pid.py`)

PID remains suitable for straight-line takeoff and emergency backup, but is **not recommended for sustained banked turns**: because `ki_roll = 0.0` (intentional to avoid wave integrator windup), the proportional roll loop (`kp_roll = 1.915`) must sit at a steady roll-angle error to generate the anti-overbanking differential flap deflection. Consequently, PID systematically **overbanks by `+11.4%`** (`8° → 8.91°`, `15° → 16.67°`, `20° → 22.13°`) and suffers `2.2–3.6×` larger turn height drops (`67.4 mm` at `8 m/s, 15°` vs `18.8 mm` on LQR).

### 5.3 Kalman Estimator Accuracy in Turns (`hydrofoil/estimation/kalman.py`)

Testing `KalmanEstimator` + `HeaveFilter` with noisy sensors at `u = 8.0 m/s`:
- **Calm turns (`8° / 12° / 15°`):** `v_err = -0.002 / -0.005 / -0.006 m/s`, `r_err = -0.02 / -0.01 / -0.02°/s`, `z_rms = 4.5–7.7 mm`, `phi_rms = 0.06°`. The linear estimator tracks steady rudderless turns accurately up to `15°` without needing a nonlinear EKF.
- **Oblique `0.15 m` wave turns (`8° / 12° / 15°`):** `v_err = -0.028..-0.035 m/s` (expected unmodeled lateral wave orbital velocity), `r_err = ±0.01°/s`, `phi_rms = 0.07–0.08°`.
- **Hardware caveat:** `sensors/models.py` currently adds zero-mean Gaussian noise to true Euler angles (`phi, theta`). On a real boat, a cheap accelerometer-aided AHRS tilts its gravity estimate toward the centripetal acceleration $u r \approx 0.8\text{ m/s}^2$ (`~4.7°` apparent roll error at `8 m/s, 15°`) unless it performs **centripetal acceleration compensation** ($a_y - u r$) using GNSS/water speed + gyro. Ensure the onboard IMU/INS has GNSS-aided or speed-compensated AHRS enabled before sea trials.

### 5.4 Supervisor, Monitor & Viewer Integration

- **`control/supervisor.py`:** `roll_limit` raised from `15°` to `20°` (giving `5°` gust headroom above `bank_max = 15°`), `safe_bank_limit(cstate)` dynamically clamps commanded bank across speed/height/waves, `bank_ref` ramps smoothly at `12°/s` (`24°/s` in `EMERGENCY_LANDING`), `_h_min` tracks active banked clearance, `TAKEOFF → FOILING` enforces pitch/roll/pitch-rate hysteresis before tightening the pitch envelope, `attitude_time = 0.25 s` debounces brief single-tick noisy wave-crest excursions on soft foiling limits (while tripping instantly on hard ceilings `25°` roll / `14°` pitch), and `status` exports `bank_ref` and `bank_max`.
- **`monitor.py`:** `breach` alert now monitors `min(center, outer wingtip)` submergence (`< 2 c` caution, `< 1 c` danger), and a new `sideslip` alert flags `|beta| > 5°` (caution) and `> 8°` (danger, approaching `8.5°` strut ventilation).
- **`viewer.py`:** pilot bank range widened to `±15°` (bounded dynamically by `safe_bank_limit` when `--supervisor` is enabled), and `api.telemetry` now reports `yaw`, `yaw_rate`, `sideslip`, and `bank_max`.

---

## 6. Open Engineering Decisions & Next Steps

1. **Wait for teammate CAD `.STL` & Flow Simulation polar** before altering wing `AR` (`3.93` vs `7.86`) or NACA 0009 section polars; then walk through the priority replacement list in [22-assumptions-and-provenance.md](22-assumptions-and-provenance.md) §12.
2. **Turning radius vs rudder decision:** with the current rudderless split-front-flap layout, `15°` bank at `8 m/s` gives a safe turn radius of `R = 77.9 m` (`53.5 m` at `6 m/s, 12°`). If tighter slalom/buoy turns (`R < 35 m`) are required, evaluate adding a small trailing-edge rudder on `elev_strut` or trimming `main_strut_chord` once structural loads are known.
3. **AHRS centripetal compensation & ranger tilt geometry:** before on-water turning trials, verify the selected IMU/INS compensates lateral centripetal acceleration `u * r` and model the physical bow/chine slant-range geometry of the real height rangers.
4. **LQR `iphi`/`itheta` coupling & phase-aware retuning:** `hydrofoil/tuning/autotune.py` now couples `iphi`/`itheta` multipliers to `phi`/`theta` and includes both the 3D `turn` scenario and the `landing` scenario in the default `--target auto` workflow (`Rising PID -> Cruising/Turning LQR -> Falling`). Re-run `python -m hydrofoil.tuning.autotune --preset manned --target auto` after CAD/polar updates to eliminate the baseline `~1.9–2.4°` transient bank overshoot while optimizing takeoff, waves, turns, and landing together.
