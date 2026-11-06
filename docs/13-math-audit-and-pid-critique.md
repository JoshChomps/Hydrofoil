# 13 — Math Audit and PID Critique

> **Status (later the same day):** the High and most Medium findings below were fixed and the gains retuned. This document records the audit **as found**; what was changed, how, and the before/after numbers are in [14-error-register-and-fixes.md](14-error-register-and-fixes.md). Numbers quoted here (e.g. roll instability) describe the pre-fix code.

Audit of 2026-11-04. Unlike docs 01–12 (written by reading), the numbers here were **produced by running the code**
(Python 3.12, repo at the state of commit `841d501` + the viewer branch; physics files unchanged).
Scripts were throwaway and are not in the repo; each result below says how to reproduce it.

## 0. Headline findings

1. **Units bug in the roll and pitch PIDs.** The controller receives angle error in **radians** (`np.radians(phi_ref)`, `eta[3]`)
   but outputs **degrees** of flap. The gains `Kp_roll = 3.5`, `Kp_pitch = 2.0` therefore mean 3.5 and 2.0 *degrees of flap per radian*,
   i.e. ≈ 0.061 and 0.035 deg/deg. The values look like deg/deg gains that are off by 57.3×.
2. **The roll loop is not stabilising in the current model.** Tested at 4 m/s foilborne, calm water:
   - start at φ = 5°, φ_ref = 0 → φ **grows** to 14° in 15 s (commanded `d_roll` is only ~0.3°);
   - φ_ref = 4° → φ reaches 16° at 16 s, the flap saturates at 20°, and the hull drops back into the water;
   - φ_ref = 2° → φ = 6.7° at 16 s and still growing. Even φ_ref = 0.5° drifts to 1.5°.
   - Multiplying roll Kp and Kd by 5 recovers the 5° disturbance (to −0.02°); ×57 (true deg/deg) tracks an 8° bank to 7.95° with 8.8° peak.
   The README/`PROJECT_STATE` claims ("verified stable up to ~10° commanded bank", "held bank ≈ 1.3–1.5× command") **do not hold for the current code**.
   They were measured before flap effectiveness (÷4.8), finite-span, free-surface and added-mass changes landed, and gains were never retuned.
3. **Waves cannot disturb roll or yaw**, so "roll Ki diverges under waves" is untestable here. A long-crested wave travelling along +x, sampled at the
   body origin with both main halves at the same x, is perfectly symmetric: max |φ| over 90 s in waves was 0.0° for Ki = 0 *and* Ki = 0.05.
4. **Heave settles in ~60–90 s, not seconds.** Takeoff (draft → 0) is complete by ≈ 10 s and u reaches 4.0 m/s, but z error is 0.36 m at 15 s,
   0.15 m at 30 s, 0.09 m at 40 s, 0.033 m at 60 s (z_ref = 0.05). The default `t_end = 60 s` ends the run before it has settled.
   The flap trim (≈ 14.1° of ±20°) is delivered almost entirely by the integrator (13.65° of 14.1° at 60 s): there is no feed-forward.
5. **Added-mass Coriolis (Munk moment) is missing** (energy is still conserved — doc 18). Adding it moves the open-loop unstable real pole from +0.178 to +0.209 s⁻¹ and the
   phugoid-like mode from 0.98 to 0.73 rad/s (−26 %). Not catastrophic, but a measurable error in the linear model any LQR/MPC would inherit.
6. Things that **are right**: Coriolis passivity, kinematics, lift/drag directions, RK4 accuracy, free-surface formula against its stated targets (see §1).

## 1. Math verification table

| Item | Result | Evidence |
|---|---|---|
| C_RB skew-symmetric: νᵀC(ν)ν = 0 | ✅ exactly 0.0 | numeric, ν = [4,.3,.2,.1,−.2,.3] |
| M symmetric / PD, conditioning | ✅ cond(M) = 24.6 | numeric (also covered by tests) |
| Rotation matrix / J / skew | ✅ | existing tests (23/23 pass) |
| Lift direction up, drag opposes V, restoring lift on sink | ✅ | tests + hand derivation of `V̂×(a×V̂)` |
| Free-surface factor hits its stated targets | ✅ 0.850 @ h/c=3, 0.998 @ 10 (0.904 @ 3.7, 0 @ 0, 0.469 @ 1) | numeric |
| Flap authority | ✅ 10.2 N/deg per half at 4 m/s → 20° gives 202 N/half = 404 N total vs 245 N weight; min foilborne speed ≈ √(245/404)·4 = **3.1 m/s** | numeric; matches doc 03 §7 estimate |
| Heave-rate stiffness dL/dw | 698 N/(m/s) per half (≈1400 total) vs the ad-hoc `heave_damping` 50 → foil damping ≈ 28× the explicit term | numeric |
| RK4 step error | ✅ dt 0.01 vs 0.0005, 2 s, fixed commands: max state error 9.4e-6 | numeric |
| Rest draft, added mass | ✅ 0.041 m; heave 24.15 kg, pitch 3.48 kg·m² (doc 03 numbers corrected) | numeric |
| Takeoff | ✅ foilborne by ≈ 10 s, u = 4.0 m/s, cruise thrust ≈ 13–14 N (cap 50 N not binding), flap ≈ 14° | closed-loop run, calm |
| Unit consistency of PID gains | ❌ rad in, deg out | `vehicle.py:190-194`, `pid.py:152-157` |
| Munk moment | ❌ absent | `RigidBody.C` uses m, I_b only |
| Wave excitation of lateral modes | ❌ none | §0 item 3 |

### Open-loop linearisation (plant only, settled foilborne state: z = 0.058, u = 4.0, θ = −0.38°)
Finite-difference Jacobian of `physics_rhs`: eigenvalues
−82.2 (roll rate), −47.2 and −22.7 (heave/pitch short period), −0.32 ± 0.47j (lateral y/v), −0.13 ± 0.98j (x, z, u: phugoid/height mode),
−0.025, **+0.178** (lateral/roll-yaw, τ ≈ 5.6 s), and zeros for x, y, ψ.
So the bare airframe has **one unstable slow lateral mode** and no roll restoring stiffness. The roll PID has to supply stability, which it currently does not (item 2).
The 8-state (z, φ, θ, v, w, p, q, r) subsystem with the three flaps as inputs is controllable (rank 8/8), so LQR is feasible on this model.
A first trim attempt with `fsolve` returned θ = −3.45°, d_main = 29.9° (spurious): **pitch trim is non-degenerate only up to θ ↔ δ_elev**, because the main foil sits exactly at the CoG
(x = 0), so pitch attitude is set by the elevator/pitch reference rather than by balance. Use the settled closed-loop state as the linearisation point, or fix θ.

## 2. Physics critique (what is wrong or oversimplified)

Ordered by how much I expect it to matter for control work.

1. **No lateral/side force.** There is no strut or fin force model; side-slip `v` produces no hydrodynamic side force or yaw moment, and the foil dynamic pressure
   uses the full |V| including spanwise flow. A real T-foil's vertical strut gives roll/yaw damping and lets the boat actually *turn* when banked. Here a banked boat slips and yaw builds
   through gyroscopic terms only (ψ ≈ 330–350° in 20 s at 8°). This is probably the largest structural omission for roll/yaw control design.
2. **No roll/pitch hydrostatic restoring moment.** Hull buoyancy and weight both act at the body origin; the hull's centre of buoyancy 0.6 m above the CoG is ignored,
   so displacement-mode GM is zero. Also `hull_draft` ignores attitude (it uses only η_z) while the foil submergence *does* rotate with attitude — inconsistent.
3. **Added mass is incomplete and un-coupled.** Heave/pitch diagonal only, no C_A (Munk), no surge/sway/roll/yaw terms, and the 2D formula ρπc²/4·span is for an infinite strip
   (3D AR-4 panels have roughly 20 % less). Heave added mass (24 kg) ≈ the vehicle's own mass, so this matters.
4. **Flap model.** The flap is an equivalent AoA shift (0.21·δ). OK for lift in the linear range, but: no section pitching moment from camber (a 40 % chord flap gives a significant nose-down Cm),
   the same shift is applied to the drag/stall lookup, effectiveness 0.21 is unvalidated, and the flap has no hinge moment (actuator torque not modelled).
5. **No wing–tail interaction.** The elevator sits 1.2 m behind the main foil with no downwash (ε ≈ 2CL/πAR) or wake deficit — usually the dominant term in tandem-foil longitudinal stability.
6. **Finite-wing inconsistency.** Lift uses AR/(AR+2) (an e = 1 lift-slope form) per half-panel of AR 4, induced drag uses e = 0.80, and the two halves together would be an AR-8 wing
   (lateral offset 0.35 m vs half-span 0.6 m also overlap geometrically). Lift slope is probably ~15–20 % low and induced drag ~2× high versus a continuous wing.
7. **Free-surface model is a monotone exponential fit** with no Froude-number dependence; real submerged-foil lift loss depends on Fr_c and can both reduce and (low Fr) increase lift.
   Also zero lift at h/c = 0 and no ventilation/cavitation/spray model, so the sim cannot show the most common real failure (surface piercing/ventilation) except as lost lift.
8. **Polar table.** One XFLR5 polar (Re 2.7e6; real Re ≈ 6e5) for every surface, α only −10°…+17.8°, flat extension beyond (not post-stall), asymmetric ends.
9. **Waves.** Single regular wave sampled once at the body origin and frozen over the step; λ and T independent (default pair is 11 % off deep-water phase speed: 2.50 vs 2.79 m/s;
   UI can create λ = 20 m, T = 0.5 s → 40 m/s); no spectrum, no spreading, no per-foil phase, and the heave loop regulates depth below the *still-water* datum rather than height above the local surface.
10. **Hull drag** is a placeholder u|u|; at the rest draft and 4 m/s it is ≈ 43 N, close to the 50 N thrust cap, and there is no drag hump, strut drag, or spray drag.
    Thrust acts through the CoG so there is no thrust-line pitch moment.
11. **Safety clamps are part of the plant.** Hard clipping of φ, θ, linear/angular velocity, and the NaN rollback inject non-smooth, non-physical behaviour that will corrupt any
    system-identification or MPC use. Keep them in the viewer/UI layer, not in the plant used for design. Euler angles (±70° clamp) should become quaternions for aggressive manoeuvres.
12. **Constants:** ρ = 1025 (seawater) hard-coded in 5 places; inertia, mass, CG, geometry are all unmeasured.

## 3. PID method — what is actually implemented

**Structure.** Three independent SISO loops (heave → symmetric main flap, pitch → elevator, roll → differential main flap) plus a speed loop (→ thrust).
Despite the "cascade PID" wording in comments and docs, **there is no cascade**: no inner rate loop under an outer angle loop; each loop is a single parallel-form PID acting
directly on an actuator command. Setpoints for height/bank/speed are set by the pilot, and pitch by a slider.

**Algorithm** (`control/pid.py`): positional (non-incremental) discrete PID, parallel form

```
I[k] = I[k-1] + e[k]·dt                     # rectangular (forward-Euler) integral
u[k] = Kp·e[k] + Ki·I[k] + Kd·(y[k] − y[k−1])/dt        # derivative on MEASUREMENT, backward difference, no filter
u_sat = clip(u, −20°, +20°)
if u ≠ u_sat: I[k] −= e[k]·dt               # "undo" anti-windup
```
So it is a **PI-D / I-PD-style two-degree-of-freedom controller** (P and I on error, D on measurement), with fixed gains, sample time = integration step (10 ms), ZOH across the RK4 stages,
followed by delay (2 steps) → 720 °/s rate limit → first-order lag (20 ms).

**Strengths:** no derivative kick; saturation handled; reset on divergence; clear separation of controller and actuator; roll sign and heave sign conventions are consistent (roll error → restoring differential lift verified from the code path).

**Weaknesses / oversights:**
1. **Unit mismatch** (rad error → deg output) — see §0.1. Heave (m → deg) is dimensionally fine; roll and pitch are not. Fix by converting error to degrees before the PID (or multiply gains by 57.3), then **retune**.
2. **No feed-forward / trim.** The 14° trim flap must be built by the integrator (Ki = 1, Kp = 15 → ~15 s time constant), hence the 60–90 s settling. Add a speed-scheduled trim feed-forward δ_trim ≈ W / (2·q·A·CLα·k_flap).
3. **No gain scheduling on speed.** Lift ∝ V², so loop gain changes ~16× between 1 and 4 m/s; gains are valid only near the 4 m/s design point and takeoff/throttle changes shift the loop.
4. **Anti-windup freezes the integrator whenever the output is saturated, even when the error is trying to desaturate it** (standard conditional integration only blocks error that pushes further into saturation). Mild with these gains, but it delays recovery from saturation and flatly differs from back-calculation.
5. **Derivative:** raw finite difference on a noise-free signal. With real sensors this needs a filter (e.g. N-filter or low-pass) — latent now because there is no sensor noise model.
6. **Surge loop sign:** the derivative-on-measurement trick assumes error and measurement move the same way; for surge `e = ref − u` they move oppositely, so `Kd > 0` is positive feedback. Dormant at Kd = 0 (UI allows up to 20).
7. **No coupling treatment.** ~~Heave and roll share a common ±20° limit, so heave demand can steal roll authority.~~ *[corrected, doc 18]* Wrong: each virtual channel is clamped separately and the plant adds them, so a physical flap can reach 40° (measured 23.5° with main saturated); the real error is that no per-surface limit exists (fixed, doc 17 §7).
8. **No reference shaping.** Setpoint steps go straight into P and I; no rate limiting of height/bank commands (the pilot UI ramps them, the PID does not).
9. **Pitch has a hidden degeneracy** (main foil at the CoG): θ_ref and δ_elev are tied only through the elevator's AoA, so the pitch loop is the only thing fixing attitude and there is no incidence-based trim.
10. **Gains were set with a different plant** (before the flap-effectiveness change); `PROJECT_STATE` "Roll gains reached by sweeping" is no longer true of the code. Everything needs retuning.
11. **The controller sees exact state** (no sensor, delay, noise or estimator), so closed-loop results are optimistic.
12. **No formal margins.** There is no loop-shape, gain/phase margin, or step-response metric in the repo; "stable" has meant "did not diverge in a throwaway script".

## 4. Recommended order of fixes (small → large)

1. Convert PID error to degrees (or scale gains), then retune roll/pitch/heave; re-run the bank sweep and disturbance-recovery tests *as committed tests*.
2. Raise `t_end`/add a speed-scheduled feed-forward trim so settling is seconds, not a minute.
3. Add wave directionality/phase per foil, lateral wave components, and a physical dispersion lock (derive T from λ) so roll and yaw can actually be excited in waves.
4. Add strut side force/yaw damping and a buoyancy lever arm so roll/yaw stability is physically modelled.
5. Add C_A (Munk) and wing–tail downwash; recompute the linear model.
6. Only then linearise and design LQR (the 8-state subsystem is controllable), with actuator augmentation and an observer.

## 5. How to reproduce

All scripts import the package from the repo root with `sys.path.insert(0, repo)`:
- *Lift, trim, eigenvalues, controllability, takeoff log:* build `VehiclePhysics(dict(DEFAULT_PARAMS))`, finite-difference `physics_rhs` (step 1e-6), `np.linalg.eig`.
- *Bank/disturbance tests:* run 60 s from `rest_state()` in `CalmWater`, then set `params["phi_ref"]` (or `state[3]`) and run 15–20 s; scale `kp_roll`, `kd_roll` to test the units hypothesis.
- *Munk:* subclass `RigidBody.C` adding `−S(M_A·ν)` blocks with `M_A = diag(0,0,m_a,0,I_a,0)`.
- *RK4:* integrate fixed commands at dt 0.01 and 0.0005 for 2 s and compare.
