# 05 — Constraints and Limits

## 1. Hard clamps in code

| Where | Constraint | Effect |
|---|---|---|
| `vehicle.step` | |φ|, |θ| ≤ 70° | Keeps clear of Euler singularity at ±90° (`T_euler` would raise `ValueError`) |
| `vehicle.step` | |u|,|v|,|w| ≤ 15 m/s | Velocity cap |
| `vehicle.step` | |p|,|q|,|r| ≤ 3 rad/s (~172 °/s) | Rate cap |
| `vehicle.step` | non-finite state → roll back to previous state, zero ν, reset controllers and actuators | Prevents NaN reaching the viewer; pilot regains control |
| `pid.HydrofoilController` | each PID output ±20° | Flap command limit (before actuator) |
| `surge` | thrust ±`max_thrust` (50 N) | Thrust can be **negative** (reverse) |
| `PID.update` | integrator step undone when output saturated | Conditional anti-windup |
| `Actuator` | rate limit 720 °/s; lag τ 0.02 s; 2-step delay | Flap response limits |
| `FoilTables` | α clamped to −10°…+17.8° | No post-stall, and **asymmetric**: negative side stops at −10° (open, doc 14 O5) |
| `Foil` | |V_foil| < 0.01 m/s → zero force | Avoids divide-by-zero |
| `hull_draft` | 0 ≤ draft ≤ `max_draft` | Buoyancy saturates at 1810 N (defaults); cannot sink beyond |
| `PilotUplink` | height −0.3…1.0, speed 0…10, bank ±8 | Applied on receipt |
| Dashboard JS | height −0.3…1.0, bank ±10 (stick), sliders see doc 04 | Dashboard allows 10° bank; uplink only 8° |
| `T_euler` | |cos θ| < 1e-6 → `ValueError` | Raw transform would crash |
| `stream.send` | `OSError` swallowed | Dropped packets are silent |

## 2. Stability limits: upstream claims vs. my re-test (see doc 13)

> **Update 2026-11-04 (superseded):** the paragraph below describes the state *before* the fixes. After the unit fix, struts and retune the roll loop is stable (see the tables at the end of this section). Original text: re-running these on the current code contradicts the upstream claims. With the current gains the roll loop does not stabilise a 5° disturbance (grows to 14° in 15 s) and a 4° bank command ends with the hull back in the water. The wave-based Ki test cannot excite roll at all (φ stays 0). The ±8° cap is therefore *not* a verified safe region today; the list below is the upstream record.

1. **Roll Ki must be 0.** Ki = 0.05 diverged to 1000+° within 90 s under waves. Gyroscopic roll↔yaw↔pitch coupling in `C_RB` outruns an integral term.
2. **Roll gains** Kp 3.5, Kd 2.5 found by sweep; old Kp 1 / Kd 1 converged to a wrong ~40° list under disturbance.
3. **Commanded bank**: held bank settles at ≈ 1.3–1.5× command (P-dominant, no integral) while yaw builds (a coordinated turn).
   Command ≥ 12° diverges to overflow in ~6 s even in calm water; verified OK up to ~10°. Hence ±8° caps. **(Not reproduced on current code; see the update above.)**
4. **Rapid multi-axis combined input** (bank + height + speed changing at once) can still diverge to NaN/Inf within 15–60 s of randomized input, even under the 8° cap.
   Not fixed at controller level; the safety net masks it. Explicitly scoped as a MIMO (LQR/MPC) problem.
5. Do not widen the viewer's `BANK_MIN/MAX` or the dashboard `ROLL_MAX` without re-running the bank sweep.

### Re-test results

**Before the fixes (audit, old gains read as °/rad, no struts):**

| Test | Result |
|---|---|
| φ₀ = 5°, φ_ref = 0 | φ grows to 14.2° in 15 s; d_roll only ~0.3–2.4° |
| φ_ref = 0.5° / 1° / 2° / 4° (16 s) | φ = 1.5° / 3.1° / 6.7° / 16.4°; at 4° flap saturates and the hull touches down |
| φ_ref = 4…15° (20 s) | φ_final 16…28°; hull back in the water; ψ ≈ 280–350° |
| Roll gains ×57 | recovers; bank 8° → 7.95° |
| Waves A 0.05, λ 5, T 2; roll Ki = 0 and 0.05 | max |φ| = 0.0° in both (waves could not excite roll) |

**After the fixes (doc 14; units fixed, struts added, gains retuned; calm unless stated, 4 m/s):**

| Test | Result |
|---|---|
| Takeoff from rest | foilborne in 2.9 s (was 9.9 s); height within 2 cm in ≈ 17 s (was 62 s) |
| φ₀ = 5°, φ_ref = 0 | back within 0.5° in 0.7 s |
| φ_ref = 8° | 8.45° steady, within 1° in 0.55 s, height loss 1.4 cm, no touchdown, heading changes |
| θ kick 3° | within 0.5° in 0.56 s |
| Height step to 0.35 m hull height | within 2 cm in 8 s |
| Waves A 0.05 m, λ 5 m: head / beam / oblique 45° | max |φ| = 0.0° / 1.0° / 1.3°, never touches down |
| Actuator delay and lag both doubled (0.04 s) | all metrics essentially unchanged (robust) |
| Fuzz / randomised combined input | **not re-run** — the upstream NaN-divergence claim is still unverified |

## 3. Operating-envelope constraints (derived from the constants; estimates)

- **Minimum foilborne speed** with current geometry ≈ 3 m/s from flap authority alone (doc 03 §7). Throttle min 0 m/s means "no lift, sink" by design.
- Flap authority: main foil flap ±20° × 0.21 effectiveness = only ±4.2° of effective AoA. Elevator ±20° × 1.0 = ±20° effective, which would hit the polar's stall region (CL_max at 14.4°) — pitch authority is far larger than heave authority. Cruise trim needs ≈ 15° of main flap (≈ 75 %), leaving ≈ 5° of headroom for roll differential + heave transients.
- Maximum foil lift at the flap limit scales with V²; at 4 m/s ≈ 400 N vs weight 245 N, i.e. only ~1.6× margin.
- Thrust: verified the vehicle reaches 4.0 m/s and cruises on ~17 N (14 N before struts and the finite-span change); hull drag at rest draft and 4 m/s would be ~43 N, so takeoff relies on the draft shrinking.
- Wave model requires depth ≫ wavelength fraction (deep water); orbital velocity evaluated at the vehicle origin depth, not at each foil.

## 4. Modelling assumptions and what they exclude

- Single-strip foils (no spanwise distribution; each half is one point at the AC). Struts, hull, and body have no hydrodynamic force other than listed.
- Zero foil incidence relative to body; body pitch changes α only through the resulting velocity direction.
- Added mass: heave and pitch diagonal only; no surge/sway/roll/yaw added mass; no added-mass Coriolis; no memory (Cummins) effects.
- No hydrodynamic damping matrix D(ν) (only foil drag, hull surge drag, and a heave-damping term).
- Single polar for all surfaces regardless of local Reynolds number (Re 2.7e6 vs actual: chord 0.15 m × 4 m/s / 1e-6 ≈ 6e5).
- Regular waves only; no current; no wind; no ventilation or cavitation; no spray.
- Hull buoyancy is a distributed hydrostatic integral over a rectangular footprint (doc 15): heave/roll/pitch stiffness match metacentric theory within 1 %; still wall-sided, no hull lines, no dynamic sinkage/trim, no planing lift or wave-making, placeholder radiation damping.
- Thrust applied at body origin (no thrust-line pitching moment; no differential thrust).
- Constant water density 1025 kg/m³ and g 9.81.
- Ground-truth full-state feedback: no sensor noise, latency (other than actuator delay), quantisation, or estimator.
- Control loop rate = integration rate (100 Hz default) and held across RK4 sub-steps.
- Real-time pacing is `sleep`-based in Python; heavy load makes sim time lag wall time (it does not catch up).

## 5. Platform / environment constraints

- Python ≥ 3.10 (tested 3.12). The MuJoCo viewer needs an OpenGL desktop and runs natively on Windows (no WSL, no UDP).
- `requirements.txt` now includes `plotly`, `mujoco` and `pytest`.
- Dashboard loads Plotly from a CDN (`cdn.plot.ly`) → needs internet.
