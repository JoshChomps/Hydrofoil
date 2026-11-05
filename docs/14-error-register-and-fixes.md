# 14 — Error Register and Fixes

Every method / unit / conversion / math error found in the audit (doc 13), expanded: what was wrong, where, the evidence, what was changed,
and which test now pins it. Placeholder *numbers* (mass, inertia, geometry, flap effectiveness 0.21, actuator constants, the new strut chord)
were deliberately left alone: they are inputs, not methods. Where a fix needed a new placeholder, it is flagged.

Status key: **FIXED** = changed and covered by a test · **FIXED (model)** = physics added · **OPEN** = not fixed (needs data or a design decision).

Branch `mujoco-viewer`. Tests: `tests/test_method_fixes.py` (new) plus the originals.

---

## A. Controller and unit errors

### E1 — Roll/pitch PID: radians in, degrees out — **FIXED**
- **Where:** `control/pid.py::HydrofoilController.update`; the caller (`vehicle.py`) passes `np.radians(phi_ref)` and `eta[3]` (rad).
- **What was wrong:** the PID `output` is a flap angle in **degrees**, but `roll_err`/`pitch_err` and the derivative measurement were in **radians**. A gain of 3.5 therefore meant 3.5 °/rad = 0.061 °/° (the numerical value looked like a deg/deg gain, 57.3× too large in intent).
  Heave (m → °) was dimensionally fine.
- **Evidence:** 5° roll disturbance → `d_roll` ≈ 0.3° and φ grew to 14° in 15 s; ×57 gain scaling made the same loop track an 8° bank.
- **Fix:** errors and measurements converted with `np.degrees` before the PID; gains are now defined as **degrees of flap per degree of error** (heave: per metre). Docstring states the units.
- **Test:** `test_roll_pitch_gains_are_degrees_per_degree`, `test_heave_gain_is_degrees_per_metre`.

### E2 — Gains out of sync with the plant — **FIXED (retuned)**
- **What was wrong:** gains were tuned for a plant that no longer exists (flap effectiveness 1.0 → 0.21, finite-span, free surface, added mass, actuator lag/delay were added later, never retuned). After E1 the *old numbers* were wrong the other way: pitch (Kp 2, Kd 2 °/(°/s)) went into a sustained limit cycle (q ≈ ±17 °/s, elevator ±10°) and any roll Kd ≥ 1 chattered (flap total variation 1750°).
- **Method:** simulate-in-the-loop sweeps on the corrected plant, each from a clean settled foilborne state (an early sweep was contaminated by the previous limit cycle in the stored controller state — noted because it is an easy mistake). Metrics: ISE after a 3° pitch / 5° roll kick, residual oscillation, flap total variation and peak, takeoff time, height settling and overshoot, hull touch-down, bank tracking, wave response (head, oblique, beam), and robustness with actuator delay and lag doubled.
- **Result (defaults in `params.py`):**

| Loop | Old (meaning) | New | Reason |
|---|---|---|---|
| heave | 15 / 1 / 5 °/m | **40 / 6 / 10** | takeoff 9.9 s → 2.9 s; 2 cm settling 62 s → ~16 s; no overshoot; ki 15 overshoots, kp 80 slower |
| pitch | 2 / 0 / 2 (really 0.035 °/°) | **1.0 / 0 / 0.1** | stable; ISE 2.1; Kd above ~0.3 only adds flap chatter |
| roll | 3.5 / 0 / 2.5 (really 0.061 °/°) | **2.0 / 0 / 0.2** | 5° disturbance rejected in ~2 s; Kd ≥ 1 chatters; Kp 3.5+ costs 13° of flap for the same job |
| speed | 30 / 5 / 0 | unchanged | cruise thrust ~17 N, not saturated; Kd sign fixed (E4) |

Roll Ki is still 0 (no integral needed; waves can now excite roll, see E12, and the loop is stable). Ki was not re-opened for roll or pitch.

### E3 — Anti-windup froze the integrator at the wrong time — **FIXED**
- **Where:** `control/pid.py::PID.update`.
- **What was wrong:** `if output != clamped: integral -= e*dt` undid the integration step whenever the output was clamped, even when the error was pulling the output *out of* saturation, which delays recovery and mis-handles sign reversals.
- **Fix:** conditional integration — freeze only while saturated **and** the error pushes further into saturation; otherwise integrate normally.
- **Test:** `test_anti_windup_keeps_integrating_when_error_desaturates`, `test_anti_windup_freezes_integrator_when_error_pushes_deeper`.

### E4 — Surge loop: derivative-on-measurement had the wrong sign — **FIXED**
- **Where:** `control/surge.py`. `PID.update` derives D from the measurement assuming error and measurement move together; for surge `e = ref − u` they move oppositely, so `Kd > 0` was positive feedback on acceleration. Latent only because Kd = 0.
- **Fix:** pass `−speed_meas` as the measurement (gives the correct `−Kd·du/dt`).
- **Test:** `test_surge_derivative_opposes_acceleration`.

### E5 — Actuator delay was in steps, not seconds — **FIXED**
- **Where:** `control/actuator.py` default `delay_steps = 2`, so changing `dt` silently changed the latency (20 ms at 10 ms, 4 ms at 2 ms).
- **Fix:** `DEFAULT_ACTUATOR_PARAMS["delay_s"] = 0.02` converted to whole steps in `HydrofoilController` (`round(delay_s/dt)`); an explicit `delay_steps` still wins (legacy/tests).
- **Test:** `test_actuator_delay_is_specified_in_seconds`.

### E6 — "Cascade PID" was a misnomer — **DOCUMENTED**
Three parallel SISO PIDs (no inner/outer structure). Comments/docs now say so; a real cascade (angle → rate → flap) is a design option in the roadmap, not an error.

---

## B. Rigid-body and hydrodynamic math errors

### E7 — Added-mass Coriolis (Munk moment) missing — **FIXED**
- **Where:** `dynamics/rigid_body.py::RigidBody.C`. `M` included the heave/pitch added mass but `C` only had the rigid-body terms; the combined model `M ν̇ + C ν` was inconsistent (energy not conserved by the added-mass part, and the destabilising Munk moment `−(Z_ẇ − X_u̇) u w` absent).
- **Fix:** added `C_A` with the same skew structure as `C_RB` using `M_A = diag(0,0,m_a,0,I_a,0)` (Fossen 6.43): blocks `−S(A11 v1)` and `−S(A22 v2)`. Passivity preserved (νᵀCν = 0).
- **Effect (measured):** unstable lateral-pole +0.178 → +0.209 s⁻¹ and height/speed mode 0.98 → 0.73 rad/s before this fix was applied.
- **Test:** `test_added_mass_coriolis_munk_moment_and_passivity` (C·ν pitch component = −m_a u w).

### E8 — Finite-span and induced-drag corrections used different efficiencies — **FIXED**
- **Where:** `forces/foil.py`. Lift used `AR/(AR+2)` (the e = 1 form with a₀ = 2π); induced drag used `e = 0.80`. The two described the same wing differently.
- **Fix:** one consistent Helmbold/Prandtl form `a_3D = a_2D / (1 + a_2D/(π e AR))`, with `a_2D` taken from the polar's own slope (6.19 /rad) and the foil's `e`. Main half (AR 4, e 0.8): factor 0.667 → 0.619; elevator (AR 3): 0.60 → 0.549. Lift per degree falls ≈ 7 % on the main halves, so the trim flap rises accordingly.
- **Test:** `test_finite_span_factor_matches_induced_drag_assumptions`.

### E9 — Dynamic pressure and AoA included spanwise flow — **FIXED**
- **Where:** `foil.py` used `|V_foil|` (3-D) for `q` and the drag direction. Spanwise (y) flow on a horizontal foil produces neither lift nor dynamic pressure, so side-slip inflated the force.
- **Fix:** work in the plane spanned by the chordwise axis and the foil's normal axis; the other component is dropped. `Foil` now takes a `normal_axis` (body z for foils, body y for struts) so one correct code path serves both.
- **Test:** `test_foil_ignores_spanwise_flow`.

### E10 — No lateral force: a banked boat skidded and yawed the wrong way — **FIXED (model; placeholders)**
- **What was wrong:** with E1 fixed alone, φ_ref = 8° gave side-slip v ≈ 3.5 m/s, **negative** yaw rate (−18 °/s for a starboard bank), speed collapsing to 2.9 m/s (below the 3.1 m/s lift limit) and hull touchdown. There was no hydrodynamic side force anywhere — struts were drawn in the viewer but absent from the physics.
- **Fix:** two vertical struts (main and elevator) as `Foil`s with `normal_axis = body y`; AoA = side-slip angle; wetted span = foil depth below the local surface, clipped to the strut length, force at mid-wetted length. They give side force, sway/yaw damping and weathercock stability.
- **After:** φ_ref = 8° tracks (8.1°), v ≈ 0.05 m/s, yaw rate positive, height held, no touchdown.
- **Placeholders:** `main_strut_chord = 0.12`, `elev_strut_chord = 0.08` (new params). The steady turn rate (≈ 3 °/s at 8° bank) depends strongly on them: with stiff struts the boat is "railed"; a coordinated turn would be ≈ 19 °/s. Measure/size the real struts before trusting turn rates.
- **Test:** `test_vertical_strut_side_force_is_restoring`, `test_bank_command_is_tracked_without_losing_height`.

### E11 — Hull draft ignored attitude — **FIXED**
- **Where:** `forces/hull.py::hull_draft` used `eta[2] − hull_offset`; the foil submergence already rotated by attitude. Now the hull-bottom point (0,0,−hull_offset) is rotated: `z_bottom = η_z − hull_offset·cosφ·cosθ`.
- **Test:** `test_hull_draft_follows_attitude`.

### E12 — Hull buoyancy acted through the CoG — **FIXED (superseded by E20)**
- **What was wrong:** the centre of buoyancy (≈ hull bottom + half the draft, above the CoG) was ignored, so there was zero hydrostatic roll/pitch stiffness in displacement mode.
- **Fix:** force applied at `r_cb = (0,0,−(hull_offset + draft/2))`, moment `r_cb × F`.
- **Test:** `test_hull_buoyancy_gives_restoring_roll_moment`.

---

### E20 — Lumped hull buoyancy had no waterplane-inertia term (roll 23 % low, pitch 14× too low) — **FIXED**
- **Found by:** cross-referencing the hull against classical hydrostatics (doc 15): heave stiffness ρ g A_wp was exact, but roll/pitch stiffness should be ρ g ∇ GM with GM = (CB above CoG) + I/∇. The lumped model only had the first term (and used one lever arm for both axes). At the 4 cm rest draft of the 2.0 × 0.30 m hull: roll 152 vs 197 Nm/rad, pitch 152 vs 2163 Nm/rad.
- **Fix:** `HullHydrostatics` integrates draft over an 11 × 7 grid of footprint cells with the full attitude kinematics and per-cell wave elevation (doc 15 §3). New params `hull_length` (2.0), `hull_beam` (0.30), `hull_x` (0.0) replace `waterplane_area` (= length × beam = 0.6, unchanged); the hull geometry in the viewer follows them.
- **After:** roll 196.5 (−0.5 %), pitch 2146.6 (−0.8 %) vs theory; force exactly linear in draft; partial emergence handled.
- **Consequence:** the correct stiffness makes a floating hull ring at ≈ 2.4 Hz with almost no damping, so a **placeholder** `hull_rot_damping` (400 N·m·s/rad × wetted fraction, zero when foilborne) was added; it stands in for radiation damping.
- **Tests:** `test_hull_hydrostatic_stiffness_matches_metacentric_theory`, `..._linear_in_draft...`, `..._rolled_partially_emerged...`, `test_floating_hull_rocking_is_damped`.
- **Controller impact:** none in flight (hull dry); gains re-checked, all foilborne metrics unchanged.

## C. Wave model errors

### E13 — Waves sampled once, at the body origin — **FIXED**
- **What was wrong:** elevation and orbital velocity were evaluated at the CoG only. Both main halves (same x), the elevator (1.2 m aft), and the struts all saw one phase; foils up to 0.5 m deeper saw the CoG's orbital decay.
- **Fix:** `VehiclePhysics.wave_sample` evaluates elevation, local surface z and orbital velocity at **each surface's own world position** (including its depth for the `e^{kz}` decay), via a `WaveSample` object passed to `physics_rhs`. (Still frozen across the four RK4 stages of a step, as before.)
- **Test:** `test_beam_seas_load_left_and_right_foils_differently_head_seas_do_not`.

### E14 — Waves could not excite roll or yaw — **FIXED**
- **What was wrong:** only a head/following sea (direction 0) existed, where left/right are identical by symmetry, so the "roll Ki diverges under waves" test was vacuous (max |φ| = 0.0° for both Ki values).
- **Fix:** `wave_direction` parameter (degrees, travel heading; UI slider −180…180, viewer `--wave-dir`). Beam/oblique seas now load the two halves differently.
- **After:** oblique 45° sea, A 0.05 m: max |φ| 0.9° in the final evaluation; the `test_oblique_waves_do_not_destabilise_roll` regression test is in the suite.

### E15 — Wavelength and period were independent — **FIXED**
- **What was wrong:** `λ = 5 m, T = 2 s` has phase speed 2.50 m/s vs 2.79 m/s from deep-water dispersion (ω² = g k); the UI could produce λ = 20 m with T = 0.5 s (40 m/s).
- **Fix:** `AiryWave(period=None)` derives `T = 2π/√(g k)`; the period slider was replaced by direction. An explicit `period` still overrides.
- **Test:** `test_wave_period_follows_deep_water_dispersion`.

---

## D. Bookkeeping, consistency and performance

### E16 — Reported state/draft were one step stale — **FIXED**
`step()` computed `draft` from the pre-step η; the dashboard published pre-step η, ν. Both now use the state at the new time.

### E17 — `sim.py` was a drifted copy of the physics — **FIXED**
It carried its own model (flap effectiveness 1.0, speed pinned to 4 m/s, different gains, constant-volume buoyancy). It is now a thin wrapper over `VehiclePhysics` (same plant, same gains), with `cmds_override` for the open-loop case and `T_END = 90 s`. Previously `T_END = 20 s` ended before takeoff settled.

### E18 — Duplicated physical constants — **FIXED**
ρ = 1025 and g = 9.81 were hard-coded in six modules; now `hydrofoil/constants.py` (`RHO_WATER`, `G`).

### E19 — Performance (needed after adding surfaces) — **FIXED**
`np.cross` and a three-matmul rotation matrix dominated the step (≈ 8.6 ms/step after adding the struts — slower than real time). `_cross` helper, closed-form `R_body_to_ned` (tested equal to the matmul form), and the identity `V×(a×V) = a − (a·V)V` bring it to ≈ 3.6 ms/step (while other jobs were running).
`test_closed_form_rotation_matches_matmul_form`.

---

## E. Still open (not errors in method — need data or a design decision)

| # | Item | Why not fixed |
|---|---|---|
| O1 | No section pitching moment from the flap (camber Cm) | needs flap section data |
| O2 | No wing–tail downwash / wake | needs a lifting-line or empirical ε model; tandem-foil stability depends on it |
| O3 | Added mass is the 2-D strip value (≈ 20 % high for AR 4); no surge/sway/roll/yaw added mass | needs panel/CFD or tow data |
| O4 | Free-surface lift loss is an exponential fit with no Froude dependence | placeholder (documented) |
| O5 | Single polar at Re 2.7e6 (real ≈ 6e5), table −10…+17.8° with flat extension | needs a polar at the right Re and post-stall data |
| O6 | Heave trim still built by the integrator (settling ~16 s, was 62 s); a speed-scheduled feed-forward would make it seconds | a controller design addition, not an error |
| O7 | Safety clamps are inside the plant; Euler angles | design decision (keep out of the design model; quaternions for MPC) |
| O8 | Controller sees exact state; no sensor/estimator | `sensors/`, `estimation/` are empty |
| O9 | Strut chord, hull drag, mass, inertia, geometry, actuator constants are guesses | measurement |
| O10 | Heave regulates depth below the still-water datum, not height above the local wave surface | sensor-model decision |

---

## F. Unit and conversion map (the audit of "everything crossing a boundary")

| Quantity | Where it lives | Unit | Conversion point |
|---|---|---|---|
| η angles φ θ ψ, ν rates p q r | state vector | rad, rad/s | `np.degrees` in the controller, dashboard, viewer text |
| `phi_ref`, `theta_ref`, `wave_direction`, `flying_height`(m) | params / UI | deg (m for height) | `np.radians` in `VehiclePhysics.step` (angles), dashboard (wave direction), viewer |
| PID angle gains | controller | **° flap per ° error** | error converted rad → deg in `HydrofoilController.update` (**E1**) |
| Heave gain | controller | ° flap per metre | none needed |
| Speed gain | controller | N per m/s | none needed |
| Flap angles δ | controller → foil | deg | multiplied by `flap_effectiveness`, added to AoA in deg; `np.degrees(atan2)` for AoA |
| Polar tables | `FoilTables` | α in deg | AoA computed in deg before lookup; slope per deg → per rad (`× 180/π`) for the 3-D correction |
| Actuator rate limit | `Actuator` | deg/s × dt | deg per step |
| Actuator delay | params | **seconds** → steps | `round(delay_s/dt)` (**E5**) |
| Viewer hinge angles | MuJoCo qpos | rad | `np.radians`, sign flipped (documented) |
| Wave direction | `AiryWave` | rad | UI degrees → rad |
| ρ, g | `constants.py` | kg/m³, m/s² | single source (**E18**) |
| Unity/UDP angle units (removed) | — | rad out / deg in | gone with the UDP bridge |

---

## H. Verification summary (final configuration, calm water unless stated, run 2026-11-04)

| Metric | Before (old gains, unit bug; audit) | After (this pass) |
|---|---|---|
| Foilborne time from rest | 9.9 s | **2.9 s** |
| Height within 2 cm of target | 62 s | **≈ 17.4 s**, no overshoot |
| Roll 5° kick → within 0.5° | diverged (14° at 15 s) | **0.69 s** |
| Bank 8° command | hull back in the water, φ 21° | **8.45°**, within 1° in 0.55 s, height loss 1.4 cm, no touchdown |
| Pitch 3° kick → within 0.5° | limit cycle (not settling) with unit-fixed old gains | **0.56 s** |
| Height step (hull to 0.35 m) | n/a | within 2 cm in 8 s |
| Waves A 0.05 m, λ 5 m: head / beam / oblique 45° max \|φ\| | 0 / 0.6 / 48° (oblique, before struts) | 0.0° / 1.0° / 1.3°, no touchdown |
| Actuator delay and lag doubled | n/a | metrics unchanged (≤ 0.05 s / 0.1° differences) |
| Tests | 23 | **46 pass** (≈ 1–2 min) |
| Physics step cost | ≈ 4 ms | ≈ 3.6 ms (8.6 ms before optimisation, with the extra surfaces) |

Caveats: the "before" oblique-wave figure is from the first corrected-units run before struts existed, not from the original code (which could not be excited by waves at all). The old pitch limit cycle is the old Kp 2 / Kd 2 read as °/° on the corrected plant. Randomised combined-input fuzz was not re-run.
