# 24 — Independent Dual Front Flaps, Physics × Control-Stability Jacobian Allocation & Top-Down Turning Safety

This document details the dual-front-flap control extension, covering why pure lift inversion cannot govern a coupled 6-DOF hydrofoil, how the $4 \times 3$ angle-normalized Jacobian QP allocator works in `< 0.4 ms`, the 3-layer Physics × Control-Stability safety envelope, stability-gated reverse drive, the top-down turning safety envelope, the singularity/hardcode audit fixes, and full verification results.

---

## 1. Engineering Rationale: Why a Jacobian QP Allocator Instead of a Static Lift Equation

Using a single lift formula $L = \frac{1}{2}\rho V^2 S C_L(\alpha, \delta)$ to govern independent flaps fails in closed-loop flight for five physical reasons:

1. **No Dynamics, Inertia, or Latency:** Static lift inversion ignores rigid-body inertia ($I_{xx}, I_{yy}, I_{zz}$), fluid added mass, pitch/roll coupling rates, loop latency (`10 ms`), and servo slew limits (`150°/s`).
2. **Coupled 6-DOF Wrenches:** Deflecting left and right flaps simultaneously moves vertical lift ($F_z$), roll torque ($M_x$), pitch moment ($M_y$), drag/yaw torque ($M_z$), and Coriolis roll–yaw–pitch coupling.
3. **Panel Asymmetry:** In oblique waves, banked turns, off-center rider loads, or partial breach/ventilation, left and right flaps operate at different speeds $V_i$, depths $h_i/c$, and angles of attack $\alpha_i$.
4. **Hinge Torque & Cavitation:** Near $9.5\text{ m/s}$ or shallow submergence, excessive flap deflection stalls the `28 N·m` gearbox limit or trips local blade cavitation before achieving higher lift.
5. **Over-Actuation & Reconfiguration:** A single-piece failure or saturation (e.g., jammed right flap) requires reallocating roll and pitch across the remaining surfaces automatically rather than blindly dividing lift.

Taking the **local Jacobian** $B = \partial [F_z, M_x, M_y, M_z]^T / \partial [\delta_L, \delta_R, \delta_E]^T$ directly differentiates the full nonlinear lift, induced/separation drag, hinge-pitching, and lever-arm equations around the current achieved drive angles $\mathbf{u}_{\text{cur}}$ while keeping physical flap angles $[\delta_L, \delta_R, \delta_E]^T$ as the constrained decision variables.

---

## 2. Independent Front-Flap Hydrodynamics (`hydrofoil/forces/foil.py`, `vehicle.py`)

### 2.1 Fast Local Derivatives & Polar Interpolation

- `FoilTables` precomputes sorted Python breakpoint lists and interval slopes `_dcl`, `_dcd` using `bisect.bisect_right`, replacing scalar `np.interp` / `np.clip` calls with O(log N) scalar interpolation (~20× speedup).
- `Foil.local_derivatives()` shares `_eval_force_and_deriv()` with `generalized_force()` (zero duplicate math) and returns analytical derivatives `dtau_ddelta` with respect to physical flap angle `delta` [per deg], accounting for:
  - Plain-flap trailing-edge separation attenuation above `flap_stall_deg` (`15°`) via `effective_flap_and_slope()`,
  - Section slope `dCL/dalpha`, Prandtl/Helmbold finite-span factor, exponential free-surface attenuation, and breach ventilation factor `(1 - 0.55 * vent)`,
  - Induced drag slope $\partial C_{D,i}/\partial\delta = (2 C_L / \pi AR e)\, \partial C_L/\partial\delta$ + separation drag derivative, creating exact differential-drag yaw torque ($M_z$).

### 2.2 Active Physical Cavitation Model (`cavitation_model=True`)

Added shared helpers `cavitation_number(speed, depth_m)` and `cp_min_and_slope(aoa_eff_deg, delta_deg, is_flap, daoa_ddelta)`:
$$\sigma = \frac{p_{\text{atm}} + \rho g h - p_{\text{vap}}}{\frac{1}{2}\rho V^2},\qquad |C_{p,\min}| \approx 0.35 + 0.12|\alpha_{\text{eff}}| + 0.012\alpha_{\text{eff}}^2 + c_\delta |\delta|$$
where hinge-corner suction $c_\delta = 0.03/\text{deg}$ applies to plain flaps (`flap_effectiveness < 0.99`) and $0$ for the all-moving rear elevator, with attached-flow suction capped at $|\alpha_{\text{eff}}| \le 18^\circ$ and $|\delta| \le 30^\circ$ so low-speed wave orbital angles ($\sim \pm 90^\circ$ before takeoff) never trigger unphysical cavitation.

When `cavitation_model=True` (default in `VehiclePhysics`) and $|C_{p,\min}| > \sigma$, let excess $e_{\text{cav}} = |C_{p,\min}| - \sigma > 0$. `Foil._eval_force_and_deriv` applies a smooth $C^1$-differentiable lift breakdown and cavity drag increment:
$$f_{\text{cav}} = 1 - \frac{0.50\,e_{\text{cav}}^2}{e_{\text{cav}}^2 + 2.5},\qquad \Delta C_{D,\text{cav}} = \frac{0.025\,e_{\text{cav}}^2}{e_{\text{cav}} + 0.5}$$
together with exact analytical chain-rule derivatives propagated into `dtau_ddelta`, `CD_induced`, and `flap_cm_delta`. In nominal calm cruise at `8 m/s` ($\sigma \approx 3.29 > |C_{p,\min}| \approx 0.82$), $f_{\text{cav}} = 1.0$ cleanly leaves trim untouched while protecting shallow or high-speed maneuvers.

### 2.3 Centerline Root-Vortex Cross-Coupling

When `left` and `right` front flaps deflect asymmetrically ($\delta_L \neq \delta_R$), `split_flap_cross_coupling` ($k_{cc} = 0.08$, bounded in $[0, 0.35]$) captures centerline gap tip/root vortex relief:
$$\delta_{\text{eff},L} = (1 - k_{cc})\delta_L + k_{cc}\delta_R,\qquad \delta_{\text{eff},R} = (1 - k_{cc})\delta_R + k_{cc}\delta_L$$
When $\delta_L = \delta_R$, cross-coupling vanishes identically so calm symmetric trim and LQR linearization remain exact.

---

## 3. Physics × Control-Stability 3-Layer Allocator (`hydrofoil/control/allocator.py`)

Rather than hardcoding static angle limits, permissible flap angles and reverse-drive splits are computed live at every `10 ms` tick across three intersecting layers:

### Layer 1: Physical Feasibility Limits (Hard Water & Hardware)
1. **Mechanical stops:** `±flap_stop_deg` (`±30°` front) and `±elev_stop_deg` (`±20°` rear).
2. **Command ceiling:** `±flap_cmd_max_deg` (`±20°`), `±elev_cmd_max_deg`.
3. **Asymmetric local-AoA stall ceiling:** Uses polar peak `aoa_peak_deg` (`14.4°` on NACA 0009) minus `stall_margin_deg` around local flow AoA (including `+3.0°` wing mounting incidence, wave orbitals, and downwash), so downward `+delta` saturates sooner than upward `-delta`.
4. **Speed-dependent hinge-torque ceiling ($V^2$):** Solves $|M_0 + k_h \delta| \le \text{torque\_util\_max} \cdot \tau_{\text{cap}}$ directly from the configured firmware torque limit (`28.0 N·m` manned).
5. **Closed-form cavitation ceiling:** Breaks $[\delta_{\min}, \delta_{\max}]$ at AoA/flap sign transitions and solves exact quadratic roots of $|C_{p,\min}| \le \sigma / \text{cav\_margin}$ in closed form (`< 5 µs`, zero iteration).
6. **Slew & Jam locks:** Intersects with per-tick slew `cmd_slew_deg_s * dt`, or freezes locked angle at measured encoder when `jam` is detected.

### Layer 2: Control-Stability Reserves, Separation Principle & Reverse-Drive Gate
- **Strict Separation Principle (`use_truth`):** When `sensors="noisy"`, `VehiclePhysics._servo_commands` invokes `ControlAllocator.allocate(..., use_truth=False)` with estimated `cstate`, calm wave state (`calm_sample()`), unventilated assumptions (`vent={}`), and nominal geometry (`_r_nominal`) so ground-truth rider CG shifts, true wave orbital velocities, and unmeasured ventilation never leak into control allocation. Physical `ServoDrive.update()` continues to integrate against `flows_true`.
- **Collinear T-Foil Yaw Axis (`Mz`):** On a rudderless T-foil with a centerline rear elevator, differential flap induced drag `Mz` is collinear with roll torque `Mx` (`Mz = -0.0396 * Mx` at `8 m/s` trim). The allocator therefore holds desired extra yaw torque `dw_des[3] = 0.0` (with minimal `0.05` regularizer weight) rather than injecting an opposing yaw offset that slows turn-in roll response.
- **Slew-Before-QP & T-Foil Elevator-Heave Decoupling:** `ControlAllocator` clamps `u_nom` into `[prev_deg ± cmd_slew_deg_s * dt]` *before* computing desired delta-wrench `dw_des = B_nom @ du_nom` so large multi-step main-flap takeoff steps never cause the QP to borrow transient lift from the rear elevator (`max_th` in `0.25 m` head-wave noisy takeoff drops to `~8.7°`). In addition, `B[0][2]` and `B_nom[0][2]` are zeroed inside the QP (`Fz` row, rear-elevator column) while keeping all pitch rows (`B[2][0..2]`) intact—preventing non-minimum-phase pitch-for-heave inversion (nose-down elevator chasing saturated front-flap lift during heavy off-center `300 kg` takeoff). Meanwhile, `w0` and true first-order `b0_true` / `_eval_nl` lift evaluation still include the full rear-elevator lift contribution, and outer-loop `saturated` flags compare realized commands against un-slewed `raw`.
- **Axis Priority Weights:** Normalized by $mg$, $mg y_{\text{lat}}$, and $mg |x_{\text{elev}}|$, enforcing strict hierarchy: **Heave ($F_z$) & Pitch ($M_y$) > Roll ($M_x$) > Yaw Residual ($M_z$)**.
- **Jammed-Flap Reconfiguration:** When `jam` is latched on a front flap, `ControlAllocator` redirects `u_nom` on the healthy flap to `a_jam ± 2*delta_roll` and switches weights to **Roll & Pitch dominant (`[0.05, 8.0, 8.0, 0.0]`)** so the rear elevator never fights unachievable heave lift against a locked flap during emergency landing.
- **Aerodynamic Zero-Lift & Stability-Gated Reverse Drive:** Net upward lift margin (`lift_margin`) is evaluated via exact nonlinear foil evaluations (`_eval_nl`) at candidate `u_sol` rather than linearized extrapolation. Opposing deflections are detected when lift coefficients oppose (`CL_L * CL_R < -1e-4` with `|split| > 1°`) or when panels straddle the geometric chordline across a large differential split (`u_L * u_R < -1.0` and `|split| > 9°`). This ensures normal cruise turns around slightly negative trim angles (`~ -1.7°` at `8 m/s`, positive lift on both panels) are never falsely blocked when `reverse_drive_enabled=False`. True opposing splits remain gated on:
  1. `reverse_drive_enabled` is `True`,
  2. Net predicted upward lift stays above `lift_floor_frac * mg / cos(phi)`,
  3. Rear elevator retains `>= elev_reserve_min` fraction of travel, and
  4. Differential split $|\delta_R - \delta_L|$ stays inside `reverse_drive_max_split_deg`.
  If any reserve tightens, the allocator sheds differential split **first** while protecting Heave and Pitch, and returns `saturated` channel booleans so `LQRController.sync_sent(..., sat_flags=...)` rolls back outer-loop integral windup using the exact `(ch, j)` channel-to-integrator mapping `((0, 0), (1, 2), (2, 1))`.

### Layer 3: Single Operator Preset (`safety_margin`)

| Preset | Stall Margin | Torque Cap Use | Cavitation SF | Lift Floor | Elev Reserve Min | Max Sideslip $\beta$ | Max Lateral g | Max Split | Bank Scale |
|---|---|---|---|---|---|---|---|---|---|
| `conservative` | `3.5°` | `75%` | `1.50` | `95%` | `35%` | `5.5°` | `0.22 g` | `16°` | `0.80` |
| `standard` *(default)* | `2.5°` | `85%` | `1.25` | `90%` | `25%` | `6.5°` | `0.30 g` | `24°` | `1.00` |
| `sport` | `1.5°` | `92%` | `1.10` | `85%` | `18%` | `7.2°` | `0.40 g` | `30°` | `1.05` |

### Exact 3-Variable Box QP & Speed Benchmark
`solve_box_qp_3d` solves the `3×3` system with a pure-Python Cramer/PDAS solver + exact 27-face active-set fallback and smooth low-speed blending (`u_eff <= 1.5 m/s` → pure angle bounds without dividing by $V^2$). Verified average call runtime: **`~0.34 ms/step`** (< 4% of the `10 ms` tick).

---

## 4. Top-Down Boat Turning Safety Envelope (`hydrofoil/control/supervisor.py`)

Added standalone `compute_turn_envelope()` (run both inside `Supervisor.update()` and on `VehiclePhysics.step()` when unsupervised) combining:
1. Geometric outer-wingtip submergence & inner-chine clearance in current estimated wave height $A_w$,
2. Low-speed authority taper (`bank_v_min_deg`) and allocator `lift_margin` headroom,
3. Attitude safety margin (`roll_limit - 5°`),
4. **Plant-derived steady-turn efficiency (`VehiclePhysics.turning_efficiency(u)`):** Lazily evaluates linearized steady-turn yaw sensitivity `turn_nu[4] * u / G` from `LQRController.turn_nu` or cached trim linearizations (`~0.118` on `model @ 4 m/s`, `~0.290` on `manned @ 8 m/s`) instead of a hardcoded constant:
   $$r_{\max} = \min\!\left(\eta_{\text{turn}}(u)\frac{g}{u}\tan\phi_{\max},\; \frac{a_{y,\max} g}{u},\; r_{\text{slip}}(\beta)\right),\qquad R_{\min} = \frac{u}{r_{\max}}$$
5. **Active Sideslip CBF Barrier:** Whenever $|\beta| > 0.80\,\beta_{\max}$ while foiling, actively unwinds `bank_tgt` inward toward `bank_limit * max(0.35, 1 - 0.65 * excess_frac)` (even if already parked at steady bank) and slows turn-in above `0.75 * beta_max`, keeping vertical struts clear of the `8.5°` ventilation threshold, plus optional `pilot["yaw_rate"]` / HTTP `yaw_rate_cmd` command support.

---

## 5. Singularity & Hardcoded Constant Audit Fixes

1. `control/supervisor.py`: Clamped `rhs / r` into `[-1, 1]` before `arccos` in `_solve_cos_sin`; replaced literal `12.0` low-speed bank floor with `DEFAULT_SUPERVISOR["bank_v_min_deg"]`.
2. `control/lqr.py`: Added `np.linalg.lstsq` / `pinv` fallbacks in `steady_turn_sensitivity` when edited geometry yields near-singular matrices; added allocator/safety keys to `_NON_PLANT_KEYS`.
3. `linearize.py`: Added damped Levenberg–Marquardt fallback `(J^T J + 1e-4 I)^-1 (-J^T r)` on singular Newton trim steps.
4. `vehicle.py`: Replaced hardcoded `20.0` limits in `_trim_ff` and `heave_pid` output clamps with `flap_cmd_max_deg`; replaced redundant `R_body_to_ned` matrix builds in submergence helpers with pure `math` scalars.
5. `monitor.py` & `consistency.py`: Replaced static `15°/19°` flap authority and `5°/8°` sideslip limits in `FlightMonitor` with dynamic allocator bounds and `turn_envelope` / `strut_vent_deg` limits; added consistency checks for `safety_margin`, `allocator`, `flap_cmd_max_deg`, and `split_flap_cross_coupling`.

---

## 6. Verification Summary

- **Supervisor & Flight Safety Suite (`tests/test_supervisor.py`):** `9/9` tests pass, including steep `0.25 m` head-wave noisy-sensor takeoff (`max |θ| ≈ 8.7°` with slew-before-QP pitch decoupling and `18°` post-stall `cp_min` saturation cap).
- **Dual-Flap, Allocator & Red-Team Fix Suite (`tests/test_dual_flaps_allocator.py`):** `14/14` tests pass, covering KKT/calm symmetric equivalence, allocator speed (`~0.34 ms/call` measured isolated; `< 0.60 ms` gate), zero-speed/emerged/rank-deficient singularity immunity, asymmetric stall/torque/cavitation bounds, preset monotonicity, stability-gated reverse drive + reserve shedding, jammed-flap reconfiguration, top-down turn radius/yaw-rate scaling + active CBF sideslip unwinding, closed-loop stress tests across PID/LQR/Auto, strict `sensors="noisy"` separation principle, `reverse_drive_enabled=False` `12°` cruise turn tracking, plant-derived `turning_efficiency(u)`, LQR `(ch, j)` anti-windup rollback, 3D `cg_z` / `prop_pos_z`, active $C^1$ cavitation derivative verification, legacy `sigma_min`, `design.py` cache/`flap_margin`, UI telemetry validation, and `encounter_frequency` / `IrregularWave` checks.
- **Full Regression Suite (`pytest tests/ -q`):** `148/148` tests pass across the entire repository (see also [docs/25-wave-direction-and-irregular-sea-impact.md](25-wave-direction-and-irregular-sea-impact.md)).
- **Co-dependency Audit:** `python -m hydrofoil.consistency` exits `0` with `0 FAIL` on both `model` and `manned`.
- **Dashboard Syntax:** `HTMLParser` open tag stack `[]`, `node --check` exit code `0`.
