# 05 — Constraints and Limits

## 1. Hard clamps in code

| Where | Constraint | Effect |
|---|---|---|
| `vehicle.step` | |φ|, |θ| ≤ 70° | Keeps clear of Euler singularity at ±90° (`T_euler` would raise `ValueError`) |
| `vehicle.step` | |u|,|v|,|w| ≤ 15 m/s | Velocity cap |
| `vehicle.step` | |p|,|q|,|r| ≤ 3 rad/s (~172 °/s) | Rate cap |
| `vehicle.step` | non-finite state → roll back to previous state, zero ν, reset controllers and actuators | Prevents NaN reaching Unity; pilot regains control |
| `pid.HydrofoilController` | each PID output ±20° | Flap command limit (before actuator) |
| `surge` | thrust ±`max_thrust` (50 N) | Thrust can be **negative** (reverse) |
| `PID.update` | integrator step undone when output saturated | Conditional anti-windup |
| `Actuator` | rate limit 720 °/s; lag τ 0.02 s; 2-step delay | Flap response limits |
| `FoilTables` | α clamped to −10°…+17.8° | No post-stall, and **asymmetric**: negative side stops at −10° |
| `Foil` | |V_foil| < 0.01 m/s → zero force | Avoids divide-by-zero |
| `hull_draft` | 0 ≤ draft ≤ `max_draft` | Buoyancy saturates at 1810 N (defaults); cannot sink beyond |
| `PilotUplink` | height −0.3…1.0, speed 0…10, bank ±8 | Applied on receipt |
| Dashboard JS | height −0.3…1.0, bank ±10 (stick), sliders see doc 04 | Dashboard allows 10° bank; uplink only 8° |
| `T_euler` | |cos θ| < 1e-6 → `ValueError` | Raw transform would crash |
| `stream.send` | `OSError` swallowed | Dropped packets are silent |

## 2. Empirically verified stability limits (from upstream `PROJECT_STATE.md`, not re-verified here)

1. **Roll Ki must be 0.** Ki = 0.05 diverged to 1000+° within 90 s under waves. Gyroscopic roll↔yaw↔pitch coupling in `C_RB` outruns an integral term.
2. **Roll gains** Kp 3.5, Kd 2.5 found by sweep; old Kp 1 / Kd 1 converged to a wrong ~40° list under disturbance.
3. **Commanded bank**: held bank settles at ≈ 1.3–1.5× command (P-dominant, no integral) while yaw builds (a coordinated turn).
   Command ≥ 12° diverges to overflow in ~6 s even in calm water; verified OK up to ~10°. Hence ±8° caps.
4. **Rapid multi-axis combined input** (bank + height + speed changing at once) can still diverge to NaN/Inf within 15–60 s of randomized input, even under the 8° cap.
   Not fixed at controller level; the safety net masks it. Explicitly scoped as a MIMO (LQR/MPC) problem.
5. Do not widen `BANK_MIN/MAX` (Python and Unity) or the dashboard `ROLL_MAX` without re-running the bank sweep.

## 3. Operating-envelope constraints (derived from the constants; estimates)

- **Minimum foilborne speed** with current geometry ≈ 3 m/s from flap authority alone (doc 03 §7). Throttle min 0 m/s means "no lift, sink" by design.
- Flap authority: main foil flap ±20° × 0.21 effectiveness = only ±4.2° of effective AoA. Elevator ±20° × 1.0 = ±20° effective, which would hit the polar's stall region (CL_max at 14.4°) — pitch authority is far larger than heave authority.
- Maximum foil lift at the flap limit scales with V²; at 4 m/s ≈ 400 N vs weight 245 N, i.e. only ~1.6× margin.
- Thrust 50 N vs hull+foil drag: unverified that the vehicle can reach 4 m/s; the surge Ki windup is limited by anti-windup.
- Wave model requires depth ≫ wavelength fraction (deep water); orbital velocity evaluated at the vehicle origin depth, not at each foil.

## 4. Modelling assumptions and what they exclude

- Single-strip foils (no spanwise distribution; each half is one point at the AC). Struts, hull, and body have no hydrodynamic force other than listed.
- Zero foil incidence relative to body; body pitch changes α only through the resulting velocity direction.
- Added mass: heave and pitch diagonal only; no surge/sway/roll/yaw added mass; no added-mass Coriolis; no memory (Cummins) effects.
- No hydrodynamic damping matrix D(ν) (only foil drag, hull surge drag, and a heave-damping term).
- Single polar for all surfaces regardless of local Reynolds number (Re 2.7e6 vs actual: chord 0.15 m × 4 m/s / 1e-6 ≈ 6e5).
- Regular waves only; no current; no wind; no ventilation or cavitation; no spray.
- Wall-sided linear hull buoyancy, no heel/trim buoyancy moment (buoyancy acts at the body origin, so the hull produces no restoring moment from lateral/longitudinal waterplane offset).
- Thrust applied at body origin (no thrust-line pitching moment; no differential thrust).
- Constant water density 1025 kg/m³ and g 9.81.
- Ground-truth full-state feedback: no sensor noise, latency (other than actuator delay), quantisation, or estimator.
- Control loop rate = integration rate (100 Hz default) and held across RK4 sub-steps.
- Real-time pacing is `sleep`-based in Python; heavy load makes sim time lag wall time (it does not catch up).

## 5. Platform / environment constraints

- Python ≥ 3.10 (uses `X | None` annotations at definition time in `sim_realtime.py`).
- Upstream runs Python inside **WSL2** and Unity on Windows; host IP auto-detected via `ip route show default`. On native Windows that detection fails and falls back to `127.0.0.1`.
- Unity project version 6000.3.6f1 (Unity 6.3), Input handling = Both (`activeInputHandler: 2`); `PythonSimDriver` uses the legacy `Input` API.
- Hull mesh `Mockup4_JacobRig_Merged.blend` (709 MB) is **not in the repo**; must be obtained from the club drive and placed at `unity-visualizer/Assets/Foil/`.
- `requirements.txt` omits `plotly` (needed by `viz.py`/`sim.py`) and `pytest`.
- Dashboard loads Plotly from a CDN (`cdn.plot.ly`) → needs internet.
