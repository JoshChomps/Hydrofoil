# 18 — Audit Verification: What Was Actually Wrong in the Original Code

Docs 13 (audit) and 14 (error register) list what I found wrong. This doc **re-checks each claim against the original code**,
not the fixed code, and runs it where possible. It also says plainly where my earlier claims were wrong, overstated or not about
the original at all.

- **Original code** = commit `841d501` (upstream `2af5157` plus docs only; no code changes).
- **Reproduce:** `python tools/verify_original_audit.py`. It extracts that commit to a temp folder and runs every check below in a
  separate interpreter that can only see the original package. Run on 2026-11-06.
- Placeholder **numbers** (mass, inertia, geometry, strut size, actuator constants) are not judged here: they are inputs, not
  methods. What is judged: math, units, signs, conversions, and missing physics that changes the answer.

## Verdict key

| Verdict | Meaning |
|---|---|
| **ERROR** | Wrong math, units or sign; produces wrong results at the default settings |
| **LATENT ERROR** | Wrong, but has no effect at the defaults (it bites when a user changes a setting the UI exposed) |
| **OMISSION** | Physics left out that materially changes the answer (a modelling gap, not a typo) |
| **MINOR** | Real inconsistency, small effect (≲ 10 %) |
| **NOT AN ERROR** | Naming, hygiene, a documented design choice, or not present in the original |
| **CLAIM CORRECTED** | My earlier claim was wrong or overstated; the correct statement is given |

## Summary

| # | Claim | Verdict | Evidence on the original code |
|---|---|---|---|
| E1 | Roll/pitch PID fed radians, output degrees | **ERROR** | A 5° roll kick grows to 8.7° at 10 s and 13.6° at 15 s. The same code with the gains ×57.3 (deg/deg) holds 0.0°. Pitch survives only because the plant is pitch-stable. |
| E2 | Gains out of sync with the plant | **ERROR (consequence)** | Foilborne at 8.5 s; height within 2 cm only after 70.6 s; 0.36 m error at 15 s |
| E3 | Anti-windup freezes when it should unwind | **ERROR, narrower than stated** | Stuck in saturation **indefinitely** in the constructed case (fixed PID: 0.99 s). In plain saturation it never winds up. |
| E4 | Surge derivative sign | **LATENT ERROR** | Kd = 1 with speed rising gives **+1.00 N** (positive feedback); default Kd = 0, UI allowed 0–20 |
| E5 | Actuator delay in steps | **LATENT ERROR** | 20 ms at dt 0.01 but 4 ms at dt 0.002; dt was editable in the dashboard |
| E6 | "Cascade PID" | **NOT AN ERROR** | Naming only (three parallel loops) |
| E7 | Added-mass Coriolis (Munk) missing | **OMISSION; CLAIM CORRECTED** | The Munk pitch moment is missing (−9.7 N m at u 4, w 0.1 m/s). But **energy was conserved** (νᵀCν = 0); my "energy not conserved" was wrong. |
| E8 | Finite-span vs induced-drag efficiency | **MINOR** | Lift factor 0.667 vs a consistent 0.618 (AR 4); 0.600 vs 0.548 (AR 3): lift ~7–9 % high |
| E9 | Spanwise flow in dynamic pressure | **MINOR** | +6.2 % foil force at 1 m/s side-slip; scales as (v/u)², so negligible in straight flight |
| E10 | No hydrodynamic side force | **OMISSION** | 1 m/s side-slip gives 3.7 N of side force (drag only) vs ~70 N from one strut; a banked boat just skids |
| E11 | Hull draft ignores attitude | **MINOR** | Draft 2.0 cm too shallow at 15° roll (4 cm draft); 0.37 mm at 2° pitch. The code comment claimed it rotated; it didn't. |
| E12 | Buoyancy acts through the CoG | **ERROR** | Floating at 5° roll / pitch: hydrostatic moment **0.000 N m** vs 17 / 189 N m restoring by metacentric theory |
| E13 | Waves sampled once, at the CoG | **ERROR** | Orbital velocity applied ~1.9× too strong at the main foil (e^(−kΔz) = 0.53); elevator 86° out of phase, not in phase |
| E14 | Waves cannot excite roll | **CONFIRMED; CLAIM CORRECTED** | `AiryWave` **did** have `direction`; the dashboard never set it. Head seas: φ = 0.000°; beam seas: 0.157°. The original comment "roll Ki = 0.05 blew up to 1000+ deg" is **not reproducible**: 0.000° in 90 s. |
| E15 | λ and T independent | **ERROR** | Default λ 5 m, T 2 s: phase speed 2.50 vs 2.79 m/s (dispersion gives T = 1.79 s) |
| E16 | Reported state one step stale | **MINOR** | Dashboard published pre-step η, ν with post-step t; draft from pre-step η (10 ms) |
| E17 | `sim.py` a drifted copy | **NOT AN ERROR (documented), but misleading** | Its own model: speed pinned at 4 m/s, constant-volume buoyancy, flap effectiveness 1.0, other gains, 20 s run. `vehicle.py` says it was kept "as-is" on purpose. |
| E18 | Duplicated constants | **NOT AN ERROR** | Every copy is 1025 / 9.81 (ρ in 7 files, g in 4); hygiene only |
| E19 | Performance | **NOT IN ORIGINAL** | The slowdown came from surfaces I added |
| E20 | Hull lacked waterplane inertia | **NOT IN ORIGINAL** | This was an error in **my first fix for E12**; the original had zero stiffness (E12) |
| D13-W7 | "Heave steals roll authority (common ±20° limit)" | **CLAIM CORRECTED** | The opposite is true: each channel is clamped **separately**, so a physical flap reached 23.5° and could reach 40°. There was no per-surface limit (F1). |
| D13-10 | Hull drag ≈ 43 N at rest draft, 4 m/s | **confirmed** | 43.4 N |
| F1 | Limits sit on virtual channels, before the mixer (found in doc 17) | **ERROR** | Same evidence as W7 |
| F2 | "Flap effectiveness 0.21 conflicts with thin-airfoil 0.75" (doc 17) | **CLAIM CORRECTED** | The original comment says the flap is on the **inboard half-span**. Expected ≈ 0.75 × (≈ 0.5 lift share) × viscous ≈ 0.3–0.36, so 0.21 is low but plausible. |
| F3 | My hinge model used the full 0.6 m span for the flap | **MY ERROR, fixed here** | The flap is 0.3 m (`main_flap_span_frac = 0.5`); hinge moments roughly halve; doc 17 numbers refreshed |

**Count, original code:**

| Verdict | Items |
|---|---|
| Errors | 7: E1, E2, E3, E12, E13, E15, F1 |
| Omissions | 2: E7, E10 |
| Latent errors | 2: E4, E5 |
| Minor | 4: E8, E9, E11, E16 |
| Not errors, or not in the original | 5: E6, E17, E18, E19, E20 |

**My own claims corrected:** 5. E7 (the energy statement), E14 (the wording, plus the Ki comment that cannot be reproduced), W7, F2,
and the E2 "before" figures (8.5 s / 70.6 s, not 9.9 s / 62 s).

**Error found in my own new code:** 1 (F3).

## Details

### E1 — Units in the roll/pitch PIDs (ERROR)
Code: `vehicle.py` passes `np.radians(phi_ref)` and `eta[3]` (rad); `HydrofoilController.update` forms `roll_err = phi_meas − phi_ref`
in rad; the docstring says outputs are flap angles in **degrees**. So Kp 3.5 means 3.5 °/rad = 0.061 °/°: a 5° error commands 0.31° of flap.
- Roll, original gains: φ 5.0° → 5.9 / 8.7 / 13.6° at 5 / 10 / 15 s. **Not stabilised.**
- Roll, same code with the error effectively in degrees (gains ×57.3): 0.0° at 5 s, −0.2° at 15 s.
- Pitch, original gains: a 3° kick decays (1.25° at 1 s, −0.10° at 5 s). The pitch loop is also 57× weak, but the boat is pitch-stable
  on its own (the elevator is a tail), so nothing visibly broke. The unit bug is real in both loops; only roll failed because of it.
- One could argue °/rad is a legal unit choice. It isn't the intended one here: the gain sliders were 0–20, and nothing in that range
  stabilises roll in °/rad.

### E2 — Gains vs plant (ERROR as a consequence of E1 and later model changes)
Original, from rest, calm water: foilborne at **8.5 s**, height within 2 cm after **70.6 s**, error 0.36 m at 15 s, trim flap 14.3°.
Doc 14's "before" column said 9.9 s and 62 s; the measured original values are 8.5 s and 70.6 s (doc 14 corrected). "Out of sync" is
a judgement about tuning, but the slow heave and the roll divergence are measured facts.

### E3 — Anti-windup (ERROR, but narrower than doc 14 implied)
Original rule: if the output was clipped, undo this step's integration, whatever the sign of the error.
- **Plain saturation** (large error, no other term): the integral never grows (stays 0.00), and recovery is immediate when the error
  reverses. In that common case the original is fine, which doc 13 called "mild".
- **Failure case**: the integral grows while P and D keep the output in range (plant moving the right way). Once the plant stops, the
  output saturates and the error reverses (−0.2). The original then undoes every integration step, so the integral **can never
  unwind**: still saturated after 30 s. The fixed (conditional-integration) PID leaves saturation in 0.99 s.
- **Trade-off not stated before:** conditional integration keeps integrating while the output is saturated on the *opposite* side
  (when the error points out of saturation). In a constructed case that let it build up an integral the original would not have.
  Back-calculation would avoid both; noted as an option (doc 14 E3).

### E4 — Surge D sign (LATENT ERROR)
`PID` derives D from the measurement, and its own docstring says this "requires error to vary the same direction as measurement".
For surge, `e = ref − u` varies opposite to `u`, so the D term is +Kd·du/dt. Measured: +1.00 N for Kd = 1 with u rising 1 m/s².
Default Kd = 0, so the defaults were unaffected; any user setting Kd > 0 (UI range 0–20) got positive feedback.

### E5 — Delay in steps (LATENT ERROR)
`delay_steps = 2`: 20 ms at dt = 0.01, 4 ms at dt = 0.002. Harmless at the default dt, but dt was a dashboard setting.

### E7 — Added-mass Coriolis (OMISSION; my energy claim was wrong)
The original `C(ν)` is the rigid-body C_RB only, while `M` includes heave/pitch added mass. Two separate statements:
- **Wrong (mine):** "energy not conserved by the added-mass part". Any skew-symmetric C conserves kinetic energy. The original
  C_RB is skew, so νᵀCν = 0.0 exactly, and ½νᵀMν is conserved with zero forcing.
- **Right:** the Kirchhoff/Fossen equations with M_A in M need C_A as well. Without it the Munk moment −(Z_ẇ − X_u̇)uw is absent:
  −9.7 N m at u = 4, w = 0.1 m/s (m_a = 24 kg). That is comparable to the elevator's pitch authority, so it is a material omission,
  not a bookkeeping detail.
The pole shifts quoted in docs 13/14 (+0.178 → +0.209 s⁻¹) were measured on the partly fixed model, not on the original.

### E8, E9 — Foil corrections (MINOR)
- E8: lift used AR/(AR+2), exactly the Helmbold factor with a = 2π and e = 1, while induced drag used e = 0.8. Consistent values:
  0.618 (AR 4) and 0.548 (AR 3) vs 0.667 and 0.600, so lift was ~7–9 % high.
- E9: `q` used |V| including spanwise flow: +6.2 % at 1 m/s side-slip at 4 m/s, growing as (v/u)². Negligible in straight flight; it
  mattered only because E10 let side-slip get large.

### E10 — No side force (OMISSION)
In the original physics, 1 m/s of side-slip at cruise produces 3.7 N of side force (the side component of foil drag). A single
0.12 m-chord strut, 0.5 m wetted, would give roughly 70 N. Nothing resisted side-slip, so a bank command turned into a skid.
(The doc 14 scenario "with E1 fixed alone, 8° bank gives v ≈ 3.5 m/s and the wrong yaw sign" was on a part-fixed model and is not
re-run here.)

### E11 — Hull draft ignores attitude (MINOR)
`hull_draft = eta_z − hull_offset` with no rotation, although the comment in `vehicle.py` said it shared the foils' rotated convention.
Error: 2.0 cm at 15° roll (against a 4 cm draft), 0.37 mm at 2° pitch. The bigger consequence is E12, not this.

### E12 — Buoyancy through the CoG (ERROR)
`hull_buoyancy_force` returns a force with **zero moment**, and gravity acts at the CoG too. Floating, heeled 5°: 0.000 N m about both
axes, against 17 N m (roll) and 189 N m (pitch) of restoring moment from metacentric theory for the 2.0 × 0.30 m footprint
(doc 15). The floating boat had **no hydrostatic roll or pitch stability**.

### E13 — Wave sampling (ERROR)
Elevation and orbital velocity were evaluated once, at the CoG position and the CoG's depth, and applied to every foil.
For λ = 5 m the main foil (0.5 m deeper) should see e^(−kΔz) = 0.53 of that velocity, and the elevator (1.2 m aft) is 86° out of phase.

### E14 — Waves and roll (CONFIRMED; wording corrected)
- Corrected: the original `AiryWave` **had** a `direction` argument. What was missing was any caller that set it (the dashboard built
  waves without it). So "only head seas existed" should read "only head seas were ever used".
- Measured: head seas, roll Ki 0 or 0.05: max |φ| = 0.000° over 90 s. Beam seas (direction 90°, possible in the class but not the UI):
  0.157°. Single-point sampling (E13) means both halves see the same flow, so even beam seas barely roll the boat.
- The original params comment, "Ki must stay 0 … verified: Ki = 0.05 blew up to 1000+ deg within 90 s", **cannot be reproduced** in
  this code. φ stays at exactly 0.000°, and `vehicle.py` clamps φ at ±70° anyway, so 1000° is impossible here. It likely came from an
  earlier version.

### E15 — Dispersion (ERROR)
For Airy waves in deep water ω² = g k is not optional. λ = 5 m needs T = 1.79 s; the default T = 2 s gives a phase speed of 2.50 m/s
instead of 2.79 m/s (11 % off), and the UI allowed far worse pairs.

### E16–E20 — Bookkeeping
- E16: the dashboard read `eta, nu` before calling `step()` but `t` after it, and `draft` was computed from the pre-step `eta`.
  A real 10 ms skew, cosmetic.
- E17: `sim.py` really is a separate model. Its own and `vehicle.py`'s docstrings say it was kept deliberately as a simpler check,
  so this is "misleading", not "wrong".
- E18: identical constants everywhere, so not an error.
- E19 and E20 are not about the original code.

### W7 / F1 — Where the flap limits sit (CLAIM CORRECTED → ERROR)
Doc 13 said heave and roll share a common ±20° limit, so heave "steals" roll authority. The code says otherwise: `heave_pid`
and `roll_pid` are each clamped to ±20°, each goes through its own actuator, and the plant adds them (right = main + roll).
Measured: main 20.0° + roll 3.5° → 23.5° on the physical flap, with no limit anywhere at the surface. The real error is the absence of
a per-surface limit, fixed in doc 17 §7 (mixer first, stops per drive).

### F2 / F3 — Flap effectiveness and the flap span (my claim corrected; my code fixed)
`MAIN_FOIL_FLAP_EFFECTIVENESS = 0.21  # plain flap, 40% chord, inboard half-span`. I had compared 0.21 with the full-span
thin-airfoil τ = 0.75 and called it a conflict. With the flap on only the inboard half of each panel, the lift effectiveness is
roughly τ × (lift share of that half, ≈ 0.5) × a viscous factor ≈ 0.3–0.36. So 0.21 is on the low side but is a placeholder, not an
error. The same comment exposed a real error in **my** hinge model: it used the whole 0.6 m panel span for the flap. Fixed:
`main_flap_span_frac = 0.5`, so the hinge carries 0.3 m of flap. The viewer now draws the flap on the inboard half, and the doc 17
budget is re-run.

## Not errors: design gaps listed in doc 13 §3
No trim feed-forward, no speed gain-scheduling, no reference shaping, an un-filtered derivative (no sensor noise exists), exact-state
feedback, no stability margins, and safety clamps inside the plant. These are real limitations of the design and its evaluation, but
not mistakes in the math. They stay on the roadmap (doc 10).

## What I did not re-verify
- The closed-loop scenarios on **part-fixed** models in docs 13/14 (E10's skid scenario, the pole shifts in E7), because they are not
  statements about the original code.
- Randomised combined-input fuzz (the original code's own "diverges within 15–60 s" comment).
- Anything that depends on hardware data (all placeholders).
