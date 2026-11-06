# 16 — Review Guide (approach and math)

A single entry point for a later, independent review. It does not repeat the detail of docs 01–15; it tells you **what to check, where it lives in the code,
what evidence exists, and what I am unsure about**. Everything is local and committed on branch `mujoco-viewer`; nothing has been pushed.

## 0. Suggested reading order (about 90 minutes)
1. This page, §1–§3 (conventions, equations, code map).
2. [03 math](03-math-and-physics.md) for derivation detail, [06 control](06-control-system.md) for the controller.
3. [14 error register](14-error-register-and-fixes.md) (what was wrong, what changed) and [15 hydrostatics](15-buoyancy-hydrostatics-cross-reference.md).
4. [13 audit](13-math-audit-and-pid-critique.md) for the *as-found* state, then §6–§7 here for the open questions.
5. Re-run the evidence: `pytest tests/` and `python tools/audit.py <cmd>` (§5).

## 1. Conventions and units (the thing most likely to hide bugs)
| Item | Convention |
|---|---|
| World frame | NED, **z down**. Position η = [x y z φ θ ψ]; z is CoG depth below the still-water datum (positive = deeper) |
| Body frame | x forward, y starboard, z down. ν = [u v w p q r] |
| Euler | ZYX, radians internally. R = Rz(ψ)Ry(θ)Rx(φ) (body → NED). φ > 0 = starboard down, θ > 0 = nose up |
| Where degrees appear | params `phi_ref`, `theta_ref`, `wave_direction`; flap angles δ; the PID's angle error (converted from rad inside the controller); UI/overlay text |
| Flap sign | δ > 0 ⇒ more AoA ⇒ more lift (trailing edge down). Left foil δ = δ_main − δ_roll, right = δ_main + δ_roll |
| Water density, g | `constants.py`: 1025 kg/m³, 9.81 m/s² (single source) |
| Gains | heave °/m; roll/pitch °/°, °/(°/s); speed N/(m/s) |
| Time | dt = 0.01 s; controller and integrator share it; delay specified in seconds |
Full boundary-by-boundary table: [14 §F](14-error-register-and-fixes.md).

## 2. The model as implemented (one pass, in execution order)

Per step `VehiclePhysics.step` ([vehicle.py:240](../hydrofoil/vehicle.py)):

1. **Controller** (held over the step): `z_ref = hull_offset − flying_height`; heave error `e_z = z − z_ref` [m]; pitch/roll errors `e = deg(θ−θ_ref)`, `deg(φ−φ_ref)`.
   Each PID: `I⁺ = I + e·dt; u = Kp e + Ki I⁺ + Kd (y−y_prev)/dt; clip ±20°`; integrate only unless (saturated ∧ error pushes deeper) ([pid.py:52](../hydrofoil/control/pid.py)).
   Default (`actuator_model = "servo"`): raw PID outputs → mixer → three drives `ServoDrive.update` ([servo.py](../hydrofoil/actuation/servo.py)), each integrating `J δ̈ = τ_out + M_hyd(δ) − B δ̇ − τ_f` in 1 ms sub-steps with moteus PD + current/voltage/cap limits; the plant gets the mixed-back actual angles. Legacy (`"kinematic"`): outputs go through `Actuator`: delay (`round(delay_s/dt)` steps) → rate limit (720°/s·dt) → first-order lag `α = dt/(τ+dt)` ([actuator.py:49](../hydrofoil/control/actuator.py)).
   Thrust: `SurgeController` PID on `e = u_ref − u` with measurement `−u` for D ([surge.py:20](../hydrofoil/control/surge.py)).
2. **Wave sample** `wave_sample` ([vehicle.py:161](../hydrofoil/vehicle.py)): elevation, local surface z and orbital velocity (`R·`…) at the hull cells and at each of 5 surfaces' world positions.
3. **Dynamics** `physics_rhs` ([vehicle.py:199](../hydrofoil/vehicle.py)): `ẋ = [J(η)ν ; M⁻¹(τ − C(ν)ν)]`, with
   `τ = τ_foils(L,R,E) + τ_struts(main,elev) + τ_hull(buoyancy) + τ_gravity + τ_hull_drag/damping + [T,0,0,0,0,0] + [0,0,−c_h w,0,0,0]`.
4. **RK4** with controls/wave frozen across stages; non-finite ⇒ rollback + controller reset; else clamp |φ|,|θ| ≤ 70°, |u,v,w| ≤ 15 m/s, |p,q,r| ≤ 3 rad/s.
5. `draft` = footprint-mean draft of the **new** state.

### Equations and where to check them
| # | Equation / rule | Code | Doc |
|---|---|---|---|
| E-a | η̇ = J(η)ν, J = blkdiag(R, T), T = [[1, sφtθ, cφtθ],[0,cφ,−sφ],[0,sφ/cθ,cφ/cθ]] | [kinematics.py:36,50,69](../hydrofoil/dynamics/kinematics.py) (R in closed form, tested = Rz Ry Rx) | 03 §2 |
| E-b | M = blkdiag(m·I, I_b) with M[2,2]+=m_a, M[4,4]+=I_a; solve `M ν̇ = τ − Cν` | [rigid_body.py:100,151](../hydrofoil/dynamics/rigid_body.py) | 03 §3 |
| E-c | C = C_RB + C_A. C_RB: off-diag −S(mν₁), lower-right −S(I_b ν₂). C_A: −S(a₁), −S(a₂), a₁=[0,0,m_a w], a₂=[0,I_a q,0]. νᵀCν = 0 | [rigid_body.py:126](../hydrofoil/dynamics/rigid_body.py) | 03 §3, 14 E7 |
| E-d | m_a = Σ ρπc²/4·span; I_a = Σ m_a,i x_i² | [rigid_body.py:36](../hydrofoil/dynamics/rigid_body.py) | 03 §3 |
| E-e | Foil: V_f = ν₁ + ω×r; V_n = V_f·n; V_plane = (V_x,0,0)+V_n n; α = atan2(V_n,V_x)[deg] + k_flap δ; q=½ρ|V_plane|²; CL = CL₂D(α)·1/(1+a₂D/(π e AR))·fs(h/c); CD = CD₂D + CL²/(π e AR); F = q A (CL·(a−(a·V̂)V̂)/‖·‖ − CD V̂); M = r×F | [foil.py:216](../hydrofoil/forces/foil.py) | 03 §4.1, 14 E8–E10 |
| E-f | fs(h/c) = 1 − exp(−(h/c)/1.58) (placeholder fit) | [foil.py:122](../hydrofoil/forces/foil.py) | 03 §4.1, 14 O4 |
| E-g | Strut = same function, n = +y (side-slip AoA), wetted span = clip(foil depth below surface, 0, strut length), force at mid-wetted point | [vehicle.py:217](../hydrofoil/vehicle.py) | 14 E10 |
| E-h | Hull: per cell z_i = η_z + R[2,:]·p_i; d_i = clip(z_i − z_surf,i, 0, d_max); F_i = ρ g dA d_i; f_i = −F_i R[2,:]; r_i = (x_i,y_i,−(h+d_i/2)); τ = Σ(f_i, r_i×f_i) | [hull.py:160](../hydrofoil/forces/hull.py) | 15 §3, 14 E20 |
| E-i | Hull drag −C·(d̄/d_max)u\|u\|; roll/pitch damping −c_rot·(d̄/d_max)(p,q) | [hull.py:177](../hydrofoil/forces/hull.py) | 15 §3 |
| E-j | Gravity f_ned = [0,0,mg] → body Rᵀ | [buoyancy.py:52](../hydrofoil/forces/buoyancy.py) | 03 §4.4 |
| E-k | Airy: η_w = A cos(k·s − ωt + φ₀), ω = √(g k); orbital (Vh,Vv) = Aω e^{kz}(cos, sin), NED z = −Vv; sampled per surface at its depth | [airy.py:57,62](../hydrofoil/waves/airy.py), [vehicle.py:161](../hydrofoil/vehicle.py) | 03 §5, 14 E13–E15 |
| E-l | Rest state: draft = m/(ρ A_wp) ⇒ z₀ = hull_offset + draft, θ₀ = 2° | [vehicle.py:148](../hydrofoil/vehicle.py) | 15 §1 |

## 3. Ledger of assumptions (classify before you review)

**A. Method choices — please challenge these**
1. Single-strip foils (one aerodynamic centre per surface, no spanwise loading) with an equivalent-AoA flap (0.21 δ), a Helmbold lift-slope correction and Oswald induced drag.
2. Spanwise flow ignored for lift/dynamic pressure on horizontal foils; struts modelled as vertical foils with side-slip AoA.
3. Foil forces use the **instantaneous** velocity (no unsteady/added-mass lag on lift, no Wagner/Theodorsen).
4. Added mass: heave and pitch diagonal from 2-D strips; consistent C_A; no coupling, no surge/sway/roll/yaw added mass; no hull added mass.
5. Hull: rectangular footprint, wall-sided prism, cell-integrated hydrostatics; placeholder drag and damping; no dynamic sinkage/trim.
6. Waves: single regular Airy wave, frozen across RK4 stages, sampled at surface AC positions.
7. Control: three SISO PIDs, PI-D structure, conditional-integration anti-windup, fixed gains, exact state feedback, no feed-forward.
8. Integration: RK4, dt 10 ms, zero-order hold of controls and wave data.
9. Safety clamps and Euler angles live in the plant used for design.

**B. Placeholder numbers (inputs, not methods)** — mass 25 kg, inertia (2,5,5), foil geometry/positions, flap effectiveness 0.21, strut chords 0.12/0.08,
hull footprint 2.0×0.30 centred on CoG, hull drag 20, hull rotational damping 400, heave damping 50, actuator (720°/s, 20 ms lag, 20 ms delay), max thrust 50 N, free-surface fit constant 1.58, Oswald 0.80, water density 1025.

**C. Known omissions** — flap section pitching moment; wing–tail downwash; Froude-dependent free-surface effects/ventilation/cavitation; polar at the true Reynolds number and post-stall; hull wave-making/hump/planing; sensors/estimator; currents/irregular seas; strut–foil junction interference; propeller/motor model (thrust is a force at the CoG).

## 4. Results to sanity-check against intuition (calm water, 4 m/s, defaults)
| Quantity | Value | Where from |
|---|---|---|
| Rest draft / added mass (heave, pitch) | 0.0407 m / 24.15 kg, 3.48 kg·m² | `tools/audit.py checks` |
| Hull stiffness: heave / roll / pitch | 6033 / 196.5 / 2147 (theory 6033 / 197.4 / 2163) | `tools/audit.py hydrostatics` |
| Flap lift slope (main, both halves), cruise trim flap, cruise thrust | ≈ 19 N/° (after the finite-span change), 15.1°, 17.1 N (θ = −0.30°) | evaluate / doc 14 §H |
| Min foilborne speed on flap authority | ≈ 3.1–3.3 m/s | doc 03 §7 |
| Takeoff / height settling / roll-kick / bank 8° | 2.9 s / 17 s (2 cm) / 0.7 s / tracks | `tools/audit.py evaluate` |
| Open-loop plant poles (settled, no controller) | −127.6, −74.6, −40.6, −23.5, −18.9, −0.21±0.76j, 0×3 (x, y, ψ), **+0.029**, **+0.25** | `tools/audit.py linearize` |
| Controllability (z,φ,θ,v,w,p,q,r; 3 flaps) | 8 of 8 | same |

Mode content (eigenvector, scaled states): −0.21±0.76j is x–u–θ–q (speed/height/pitch oscillation, "phugoid-like"); **+0.25 s⁻¹ (τ ≈ 4 s)** is lateral y–ψ–φ–r (spiral-type divergence, stabilised by the roll PID);
+0.029 s⁻¹ is a very slow x-dominated mode — **not diagnosed** (see §6).

## 5. Evidence and how to re-run it
| Claim | Evidence | Command |
|---|---|---|
| Unit conversions and each fix | 46 unit/regression tests | `pytest tests/` (~1–2 min) |
| Hull stiffness equals metacentric theory (<1 %) | analytic comparison | `python tools/audit.py hydrostatics` |
| νᵀCν = 0, Munk term, RK4 convergence, wave dispersion | identities | `python tools/audit.py checks` |
| Open-loop poles, controllability | finite-difference Jacobian at the settled state | `python tools/audit.py linearize` |
| Closed-loop metrics, robustness to doubled delay/lag | scenario suite | `python tools/audit.py evaluate` |
| Gain choices | sweeps | `python tools/audit.py sweep-angles`, `sweep-heave` |
| Pre-fix behaviour (for comparison) | commit `7dd84f9` (before fixes) vs `8da55b3`/`f57fd89` | `git checkout 7dd84f9` then run the same commands (note: they target the new API) |

## 6. Open questions I would like a second opinion on
1. **Foil force model validity.** One strip per surface with an equivalent-AoA flap: is the 0.21 effectiveness (40 % chord, inboard half-span) credible, and should the flap add a section pitching moment? How big is the error from ignoring unsteady lift at the encounter frequencies here (~0.5 Hz)?
2. **Wing–tail downwash** is absent. With main foil at x = 0 and the elevator 1.2 m aft, how large is ε = 2CL/(π AR) in practice and does it change the longitudinal stability materially?
3. **Free-surface correction.** Is a monotone exponential in h/c acceptable at h/c ≈ 3.7 (factor 0.90), or should it be Froude-dependent (Fr_c = U/√(g c) ≈ 3.3 here)? Ventilation limits are not modelled.
4. **Strut model.** Side force from a strip with wetted span = depth, free-surface image effect ignored. The steady turn rate (≈ 3°/s at 8° bank) depends strongly on the placeholder chords — should turning instead be driven by a rudder/differential-thrust model?
5. **Added mass.** Is it right to include only heave/pitch? Without surge/sway/yaw added mass, sway and yaw dynamics may be too lively.
6. **Hull.** Is wall-sided hydrostatics + a placeholder u|u| drag good enough for the takeoff phase? Takeoff now completes in 2.9 s — faster than I would expect from a real boat; the pre-takeoff drag hump and hull sinkage are not modelled.
7. **Control structure.** Should the heave/roll interplay (shared main-flap authority, ±20°) be handled with allocation instead of simple summation? Is it acceptable that the integrator carries the 14° trim (17 s settling) instead of a feed-forward?
8. **Anti-windup.** Conditional integration chosen over back-calculation; is that appropriate for the takeoff transient (flap saturated at 20° for ~2 s)?
9. **Frozen wave field across RK4 stages** and the 10 ms zero-order hold on controls: acceptable for dt = 10 ms? (RK4 error vs dt: 9e-6 near trim, 4e-4 for an aggressive off-trim state.)
10. **Safety clamps in the plant.** Should they be removed from the design model and moved to the UI layer before any linearisation/LQR work?
11. **Euler angles vs quaternions** for roll/pitch up to ±70° and turning flight.
12. **Drive model (now in the loop, doc 17).** Check the hinge-moment sign conventions, the thin-airfoil viscous factor 0.8, the flap-as-isolated-plate unsteady approximation, and the back-drive efficiency rule `τ_out = N τ_m/η`.

## 7. Things that look odd or are not fully explained
- The **+0.029 s⁻¹ open-loop mode** (x-dominated) was not traced to a physical mechanism; with the speed PID in the loop it is irrelevant but it should be understood.
- **Turn rate vs coordinated-turn expectation:** at 8° bank the model turns ≈ 3°/s; a coordinated turn at 4 m/s would be g tanφ/V ≈ 19°/s. The gap is the strut side force opposing the tilted lift; it depends on the placeholder strut chords (doc 14 E10).
- **Takeoff time 2.9 s** is short; it is a consequence of 50 N thrust, low hull drag (≈ 43 N at rest draft and 4 m/s but falling rapidly), the large heave gain and flaps at the 20° limit.
- **Heave PID** still converges slowly (17 s for 2 cm) because trim is integrator-built; I did not add feed-forward because that is a design change, not an error fix.
- **Strut wetted length at rest** is the full strut length (foil ≈ 1.1 m deep); strut drag and side force at near-zero speed are negligible but untested.
- Randomised combined-input fuzzing (the upstream NaN-divergence claim) was **not** re-run after the fixes.

## 8. File map of the documentation
`01` overview/status · `02` code reference · `03` math · `04` constants/parameters · `05` constraints/limits/re-tests · `06` control · `07` interfaces · `08` MuJoCo viewer ·
`09` manual · `10` roadmap · `11` known issues · `12` progress log · `13` audit as found · `14` error register & fixes · `15` hydrostatics cross-check · **`16` this guide**.
Tools: `tools/audit.py`. Tests: `tests/test_forces.py`, `tests/test_actuator.py`, `tests/test_method_fixes.py`.

## 9. Commit trail (local only)
`2af5157` upstream · `841d501` first docs · `75042b1` MuJoCo viewer + audit docs · `7dd84f9` findings folded into docs · `8da55b3` method/unit/math fixes + retune · `f57fd89` distributed hull hydrostatics · (next) review guide + tools.
