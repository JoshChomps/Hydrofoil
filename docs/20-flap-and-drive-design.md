# 20 — Main-foil flap and drive design (manned preset)

How the main foil's flap geometry and the drive's gear ratio set the hinge torque, motor current and
mechanical loads. Which mechanical constraints follow, how backlash and flap twist change things, what the
manned preset now uses, and what mechanism still has to be designed.

Numbers are for `MANNED_PARAMS`: 275 kg, 0.28 m chord × 1.1 m per half, 3° incidence, 8 m/s cruise, with the
mj5208 + moteus r4.11 drive at 40 V, unless noted. Every closed-loop number below comes from a run of
`tools/flap_design.py fly` and every static one from `tools/flap_design.py static`.

    python tools/flap_design.py static                 # sections 3, 4 and 5 (about 3 s)
    python tools/flap_design.py fly                    # the manned preset as configured (a few minutes)
    python tools/flap_design.py fly --cf 0.4 --sf 0.5 --sb 0 --slew 0 --rigid   # the pre-2026-11-16 flap and drive
    python -m hydrofoil.consistency                    # parameter co-dependency check (section 9)

## 1. Corrections to the first version of this doc (commit 3d8ad08)

The first version had these errors. All are fixed below.

| # | What it said | What is true | How it is prevented now |
|---|---|---|---|
| 1 | "The current flap does not fit the drive: 44 N·m needed at 9.5 m/s against 28.9." | 44 N·m was a static sizing assumption (±3° of effective angle-of-attack authority on top of trim), not a flight result. In closed loop at 9.5 m/s in a 0.15 m head sea, the old flap peaks at **17.3 N·m**, inside the drive. | Static figures are labelled as assumptions. The closed loop now includes a 9.5 m/s head sea (`head_top`). |
| 2 | "Holding trim needs 17–20 A, over the controller's 12 A rating." | 12 A is the moteus's figure without a heat sink. The controller runs higher (20–32 A with heat sinking; the drives sit in water). The real limit will be the firmware co-dependency limit (section 10). | `drive_i_cont` is a parameter (manned: 20 A, placeholder). The check is a thermal WARN, not a limit. |
| 3 | "Hinge moment is under 14 N·m in all runs; the takeoff peaks are the command, not the water." | True for the smaller flaps. The old 40 % flap reached **21.1 N·m** of hinge moment at takeoff. | The claim is restricted to the rows it holds for. |
| 4 | "Takeoff rail peak 17–21 A in every configuration." | **12.7–21.4 A** (12.7 A at 30:1). | Re-read against the run table. |
| 5 | "Effectiveness loss factor about 0.55." | **0.56** (0.21 / (0.747 × 0.5)). | — |

Found while doing this work:
- **Slew limiter, first step not limited.** The first version of the slew limiter took its starting point from
  the first command, so the takeoff step to 20° went through unlimited. Its test also skipped the first step.
  Both are fixed; the test now starts from the surfaces' real angle.
- **A test hard-coded the old geometry's cruise trim (−2.78°).** It now compares against the trim solver.

## 2. What changed in the model

- **Flap lift effectiveness follows the geometry** (`forces/foil.py::main_flap_effectiveness`). It scales the
  0.21 reference (40 % chord, half-span) by Glauert's τ(c_f/c) times the flapped span fraction. The 25 kg model
  is unchanged at 0.21. The manned preset is now 0.342.
- **Set-back hinge (overhang balance)**: `main_flap_setback`, a fraction of the flap chord
  (`hinge.thin_airfoil_coeffs(..., setback=)`).
- **Flap structural inertia follows the geometry** (`hinge.flap_inertia`): I ∝ L_f·c_f³·c·(1/12 + (½ − set-back)²).
  The constant is fitted to the manned placeholder 9e-3 kg·m², and the 25 kg model's 4e-4 then follows to within 1 %.
  The new manned flap is 2.7e-3 kg·m².
- **Compliant drive train** (`actuation/servo.py`). This is new; the drive was rigid before.
  - **Two masses.** The gear output (rotor reflected, J_rotor·N²) and the surface root, joined by the gear's
    torsional stiffness (`gear_stiffness`, damping ratio 0.05) with a backlash dead zone (`backlash_deg`).
  - **Spanwise twist.** The flap is driven at its root. Its span-mean, lift-producing angle is
    root + M_hinge / k_tw, with k_tw = 3GJ/L (flap) or 12GJ/L (elevator, driven at mid-span). GJ comes from
    `main_flap_GJ` / `elev_GJ`.
  - **Encoder position.** `encoder` is "motor" (the moteus's own), "output" (an absolute encoder at the surface
    root) or "dual" (output-encoder position, motor velocity for damping).
  - **Defaults are rigid**, so the 25 kg model is unchanged. A very stiff spring reproduces the rigid model
    (test).
- **Command slew limit** (`cmd_slew_deg_s`). Each drive's command is rate-limited from the surfaces' actual
  angle.
- **Co-dependency checker** (`hydrofoil/consistency.py`, section 9).

Hand checks of the drive model (tests in `tests/test_drive_compliance.py`; frozen hinge load −4 N·m −
100 N·m/rad·δ, 5° command):
- **Rigid:** 2.667°, against η·kp·(cmd − δ) = load by hand: 2.668°.
- **0.3° backlash, motor encoder:** the surface sits (b/2)·ηkp/(ηkp + k_load) = 0.102° low, and the model agrees
  to 1e-3°. An output encoder removes the offset.
- **Gear stiffness:** adds load/k_g, as expected.
- **Twist:** −0.6613° in both the model and M/k_tw.

## 3. Flap geometry: the trade (static)

Thin-airfoil coefficients (per rad; the hinge moment is referenced to q·S_f·c_f; the sim applies a 0.8
viscous factor):

| c_f/c | set-back | τ | Ch_α | Ch_δ | hinge torque for the same lift change (40 % plain = 1) |
|---|---|---|---|---|---|
| 0.20 | 0 | 0.55 | −0.50 | −0.93 | 0.31 |
| 0.25 | 0 | 0.61 | −0.57 | −0.95 | 0.45 |
| 0.30 | 0 | 0.66 | −0.63 | −0.97 | 0.61 |
| 0.30 | 0.1 | 0.66 | −0.47 | −0.68 | 0.43 |
| 0.30 | 0.2 | 0.66 | −0.30 | −0.39 | 0.24 |
| 0.30 | 0.3 | 0.66 | −0.14 | −0.10 | 0.06 |
| 0.40 | 0 | 0.75 | −0.75 | −1.01 | 1.00 |
| 0.50 | 0 | 0.82 | −0.86 | −1.07 | 1.50 |

- **Chord sets the torque.** For the same lift change, hinge torque goes as c_f²·Ch_δ/τ, and the flapped span
  cancels.
- **Span sets the deflection.** Plain flaps lose effectiveness past about 15–20°, which is **not modelled**.
- **Set-back** cuts Ch_δ by about 60 % at 20 %. Thin theory overbalances at about 33 % set-back (25 % flap: a
  0.2 limit at 0.6× that point; the checker enforces it). Real cove flow gives less benefit than thin theory.

Per main-flap drive at level trim. The **±3° effective angle-of-attack authority** is a sizing assumption, not a
measured demand (closed loop: section 6). M_pk is the trim moment plus that authority. I_hold is the motor
current that holds the trim moment at 20:1.

| c_f/c, span, set-back | speed (m/s) | effectiveness | trim flap | deflection with authority | trim moment (N·m) | M_pk static (N·m) | I_hold (A) |
|---|---|---|---|---|---|---|---|
| 0.40, 0.5, 0 (old) | 6.0 / 8.0 / 9.5 | 0.21 | 6.6° / −2.8° / −6.3° | 20.9° / 17.1° / 20.6° | −8.4 / 1.0 / 9.9 | 22.2 / 25.4 / 44.3 | 17.2 / 2.0 / 20.1 |
| 0.25, 0.5, 0 | 9.5 | 0.17 | −7.9° | 25.4° | 5.4 | 20.8 | 10.9 |
| 0.25, 1.0, 0 | 9.5 | 0.34 | −3.9° | 12.7° | 3.8 | 19.2 | 7.7 |
| 0.40, 0.5, 0.2 | 9.5 | 0.21 | −6.3° | 20.6° | 3.6 | 17.6 | 7.4 |
| **0.25, 1.0, 0.15 (adopted)** | 6.0 / 8.0 / 9.5 | 0.34 | 4.1° / −1.7° / −3.9° | 12.9° / 10.5° / 12.7° | −2.4 / −0.2 / 1.9 | 5.7 / 6.2 / 10.3 | 4.8 / 0.4 / 3.8 |
| 0.25, 1.0, 0.2 | 9.5 | 0.34 | −3.9° | 12.7° | 1.2 | 7.4 | 2.5 |

Against measured flight, the static M_pk is conservative:
- old flap at 9.5 m/s: 44.3 static, 17.3 flown;
- adopted flap at 9.5 m/s: 10.3 static, 4.0 flown.

## 4. Gear ratio

| N | full output torque (N·m) | reflected rotor inertia (kg·m²) | servo natural frequency, kp 250, old flap (Hz) |
|---|---|---|---|
| 10 | 14.4 | 0.007 | 13.3 |
| 14 | 20.2 | 0.014 | 12.2 |
| 20 | 28.9 | 0.028 | 10.6 |
| 30 | 43.4 | 0.063 | 8.3 |
| 50 | 72 | 0.175 | 5.6 |

- **Motor speed never limits N.** Back-EMF only binds above about 240:1.
- **Torque gives the lower bound:** N ≥ 1.5 × demand / (1.7 × 0.85).
- **Reflected inertia gives the upper bound.** It passes the old flap's own inertia near 20:1, and three times
  the new flap's (2.7e-3 + water 6e-3). Above that the drive is slower and harder for the water to back-drive.
- **Current falls as 1/N.** On the adopted preset at 14:1 against 20:1, head-sea rms current goes 3.4 → 5.0 A
  and the 9.5 m/s figure 4.5 → 6.6 A.
- **Keep 20:1.**

## 5. Mechanical constraints

The drive can put its full output torque into a stop, a jam or a weed strike. Size for that, or limit it.

| N | full torque (N·m) | solid output shaft (316 SS, SF 2 on shear yield) | pushrod force at a 20 / 30 / 40 mm horn |
|---|---|---|---|
| 10 | 14.4 | ≥ 10.2 mm | 720 / 480 / 360 N |
| 20 | 28.9 | ≥ 12.8 mm | 1450 / 960 / 720 N |
| 30 | 43.4 | ≥ 14.7 mm | 2170 / 1450 / 1080 N |

- **Output shaft.** The `DriveSpec` placeholder was 8 mm (4× overstressed) and is now **13 mm**. The checker
  failed on it for both presets.
- **Section thickness at the hinge line** (NACA 0009, 0.28 m chord):

  | hinge position | thickness |
  |---|---|
  | 60 % chord | 19 mm |
  | 70 % chord | 15 mm |
  | 75 % chord (adopted) | 13 mm |

  A 13 mm torque shaft does not fit along the hinge line. Torque goes in at the flap root; the spanwise hinge
  carries shear only, and the flap is a torsion box.
- **Transmission down the 1.65 m strut** (steel, stiffness at the flap compared with the servo's kp of 250):
  - 8 mm pushrod: buckles unguided at 146 N, against a 960 N load.
  - 20 × 1.5 mm tube: works (2.7 kN buckling) but only just fits a 21 mm-thick strut.
  - Torque tubes: 19–359 N·m/rad, too soft.
  - Conclusion: put the drives at the strut–foil junction (section 11).
- **Rail current.** The three manned drives are under 1.1 A at peak in every adopted-preset run (section 7).
  Propulsion (about 110 A at 40 V) needs its own battery path.

## 6. Closed-loop geometry sweep (rigid drive, no slew limit; LQR and trim feed-forward re-derived per geometry)

Scenarios:
- **takeoff**: from rest to 8 m/s, calm, 40 s.
- **head / following**: flying start, 0.15 m, 10.8 m sea, 20 s.

Main-flap drive values; the rail is all three drives.

| c_f/c, span, set-back, N | head sea height rms | head M_pk / I_rms | following M_pk / I_rms | head rail mean / peak | takeoff hinge M_pk / I_pk / rate_pk / rail peak |
|---|---|---|---|---|---|
| 0.40, 0.5, 0, 20 (old) | 18.2 mm | 7.7 N·m / 9.8 A | 6.9 / 8.0 | 0.60 / 1.3 A | 21.1 N·m / 48 A / 549°/s / 17.6 A |
| 0.40, 0.5, 0, 14 | 18.1 | 7.7 / 14.1 | 6.9 / 11.4 | 1.04 / 2.4 | 21.1 / 59 / 586 / 21.4 |
| 0.40, 0.5, 0, 30 | 18.4 | 7.6 / 6.4 | 6.8 / 5.3 | 0.36 / 0.7 | 21.1 / 32 / 475 / 12.7 |
| 0.30, 0.5, 0, 20 | 17.6 | 5.5 / 6.1 | 5.1 / 5.4 | 0.36 / 0.7 | 12.7 / 48 / 605 / 17.8 |
| 0.25, 0.5, 0, 20 | 17.5 | 4.4 / 4.5 | 4.1 / 4.1 | 0.29 / 0.5 | 9.2 / 48 / 609 / 16.9 |
| 0.25, 1.0, 0, 20 | 15.0 | 4.6 / 5.2 | 4.0 / 4.4 | 0.29 / 0.5 | 13.7 / 48 / 604 / 17.8 |
| 0.40, 0.5, 0.2, 20 | 17.9 | 3.4 / 6.6 | 2.9 / 3.8 | 0.41 / 0.7 | 9.4 / 48 / 549 / 17.6 |
| 0.25, 0.75, 0.2, 20 | 15.8 | 1.7 / 2.7 | 1.5 / 1.8 | 0.23 / 0.3 | 4.9 / 48 / 607 / 17.4 |
| 0.25, 1.0, 0.2, 20 | 14.9 | 2.0 / 2.9 | 1.7 / 1.9 | 0.22 / 0.3 | 6.5 / 48 / 604 / 17.8 |
| 0.25, 1.0, 0.2, 10 | 14.7 | 2.0 / 6.0 | 1.7 / 3.9 | 0.45 / 0.8 | 6.5 / 59 / 585 / 18.5 |

- **The unshaped takeoff step causes the takeoff current peak (48 A at 20:1).** The flap slews to its 20° limit
  at about 550–610°/s, and that saturates the drive torque for every geometry.
- **The hinge moment differs by geometry:** 21 N·m on the old flap, 5–14 N·m on the smaller ones.

## 7. Backlash, gear compliance and flap twist (adopted geometry unless noted, LQR, slew 150°/s)

| case | head sea height rms | head / 9.5 m/s height rms | takeoff I_pk / rail peak | twist peak, takeoff / cruise / 9.5 m/s | backlash gap peak |
|---|---|---|---|---|---|
| rigid, no slew limit | 14.9 mm | 14.6 mm | 48 A / 15.6 A | — | — |
| **adopted: 0.25° backlash, 4000 N·m/rad, dual encoder** | 15.2 | 15.0 | 11.1 / 1.1 | 0.64° / 0.22° / 0.34° | 0.23° |
| same, motor encoder | 15.8 | 15.6 | 11.1 / 1.0 | 0.64° / 0.22° / 0.34° | 0.23° |
| 0.5° backlash, motor encoder | 16.3 | 16.1 | 11.1 / 1.1 | 0.64° / 0.22° / 0.34° | 0.36° |
| old flap, compliant, output encoder | 18.6 | 17.9 | 32 / 16.3 | 0.86° / 0.31° / 0.71° | 1.92° |

- **The output-only encoder rings.** Position and damping both come from the far side of the gear spring,
  which is non-collocated feedback. From rest (no water damping) the motor saturated for 0.27 s and **72 N·m went
  through the gear**, 2.4× the gearbox's 30 N·m rating.
- **Dual feedback** (position from the output encoder, damping from the motor's velocity) brings that to 7.6 N·m,
  with no torque limiting. The checker now warns on output-only feedback with a compliant drive train.
- **The balanced flap makes the gear knock.** The hinge moment at cruise is only −0.26 N·m. In calm water that
  keeps the gear on one flank (0 flank changes in 25 s). In a 5 cm head sea the moment swings ±0.46 N·m, and the
  gear crosses its backlash 56 times in 25 s, twice per wave, with either encoder.
  - **Control cost: small.** Height std is 5.0 mm with the output encoder against 5.6 / 6.1 mm with a motor
    encoder at 0.25 / 0.5° backlash.
  - **Mechanical cost: tooth impacts** every wave. Mitigations: a preloaded or anti-backlash stage, a light
    spring preload on the flap, or less set-back so the hinge keeps a one-sided load.
- **Twist:**
  - Adopted flap at cruise: 0.2–0.34°.
  - Takeoff: 0.64°.
  - Old flap: up to 0.86°.

  The static checker bound, 0.87° at the 4.5 m/s schedule point, is pessimistic.
- **No limit cycles.** Settled calm-water height wander after takeoff is 0.0006–0.02 mm in every case.

## 8. Adopted manned configuration (2026-11-16)

| parameter | old | new | why |
|---|---|---|---|
| `main_flap_frac`, `main_flap_span_frac`, `main_flap_setback` | 0.4, 0.5, 0 | **0.25, 1.0, 0.15** | sections 3 and 6. 0.15 rather than 0.2 keeps 2× margin to thin-theory overbalance. |
| `main_surface_inertia` | 9e-3 | **2.7e-3** | scales with the geometry (section 2) |
| `trim_ff_deg` (at 6 m/s) | 6.57 | **4.11** | re-derived with `linearize.trim` |
| `cmd_slew_deg_s` | — | **150** | the takeoff step; cruise in waves needs about 40°/s |
| `servo_params` | rigid | **backlash 0.25°, gear 4000 N·m/rad, encoder "dual"** (placeholders) | section 7 |
| `main_flap_GJ`, `elev_GJ` | rigid | **250, 1000 N·m²** (placeholders) | carbon closed sections |
| `drive_i_cont` | 12 (DriveSpec) | **20 A** (placeholder) | the controller runs higher; firmware limit to come |
| `DriveSpec.shaft_diameter` | 8 mm | **13 mm** | section 5 |
| PID gains, LQR weights | — | **unchanged, re-checked** | see "Tuning" below |

Result, old configuration (40 % flap, rigid, no slew limit) against adopted, LQR, ideal sensors:

| | takeoff hinge M / I_pk / rail peak | head sea 8 m/s: height / M_pk / I_rms | following: height / M_pk / I_rms | head sea 9.5 m/s: height / M_pk / I_rms |
|---|---|---|---|---|
| old | 21.1 N·m / 48.4 A / 17.6 A | 18.2 mm / 7.65 N·m / 9.8 A | 18.0 / 6.88 / 8.0 | 17.5 / 17.3 / 18.9 |
| **adopted** | **7.7 / 11.1 / 1.1** | **15.2 / 2.60 / 3.4** | **14.8 / 2.26 / 2.5** | **15.0 / 4.04 / 4.5** |

- **Elevator peak:** 12.9 A at takeoff, 8.2 A in head seas.
- **Gear-train torque peaks:** 7.6 N·m at takeoff, 3.8 in head seas, 5.6 at 9.5 m/s.
- **Torque limiting:** none.
- **Checks:** 99 tests pass, including the four supervisor missions (takeoff, cruise, landing, flap jam,
  elevator loss).

**Tuning (both re-checked on the new geometry with the autotuner; neither result adopted):**
- **LQR:** the search improved the score from 9.55 to 9.16, but only with ideal sensors. With noisy sensors it
  was worse: 25 → 28 mm, flap rate 31 → 47°/s. Kept.
- **PID:** the search improved the score from 22.1 to 10.95, with kp/ki/kd_heave 340 / 264 / 19 against
  45 / 6.5 / 7.9. It **broke the supervised takeoff**:
  - below liftoff speed the supervisor holds the height reference at the hull;
  - the stiff heave loop held it with −13.5° of flap, digging the hull in;
  - the boat sat on the drag hump at 5.3 m/s at full thrust until the takeoff timeout.

  The tuner's takeoff scenario does not run the supervisor, so it could not see this. Scaling the old gains by
  the effectiveness ratio (0.21/0.342) scored worse (25.7). Kept the old gains, which pass the supervisor tests.

## 9. Co-dependency checker (`python -m hydrofoil.consistency`; `tests/test_drive_compliance.py` requires no FAIL)

| check | why it exists |
|---|---|
| flap effectiveness = f(geometry); `trim_ff_deg` = trim solver | derived values went stale silently before |
| `tuned_for` = current geometry (WARN) | gains and weights carry the geometry they were tuned on |
| torque cap ≤ drive full torque, ≤ gearbox rating; current at the cap ≤ controller peak | a cap the hardware cannot reach, or one above the gearbox |
| output shaft stress at min(cap, full torque) | **caught the 8 mm shaft (FAIL on both presets)** |
| trim exists at every LQR schedule speed; trim + authority inside the plain-flap range (WARN) | **flags the manned 4.5 m/s schedule point**: 14.6° + 8.8° authority (it was 23° on the old flap, beyond the ±20° command limit). It is below the 5.6–6.3 m/s liftoff, so the LQR never flies there (engage at 4.8 m/s, hull clear). |
| static hinge demand × 1.5 ≤ cap; holding current ≤ `drive_i_cont`; rail at holding (WARN) | sizing margins; labelled as static assumptions |
| hinge stiffness < servo kp; gear stiffness ≥ 5 kp; encoder vs backlash; output-only feedback with compliance (WARN) | servo-loop interactions from section 7 |
| flap twist at the worst demand (WARN) | section 7 |
| `main_surface_inertia` matches `flap_inertia` within 30 % (WARN) | inertia did not follow the geometry before |
| set-back not overbalanced (FAIL) and ≤ 0.6× the overbalance point (WARN); elevator pivot ahead of its aerodynamic centre | hinge balance |
| flap command limit ≤ stop; slew ≤ drive no-load rate | commands against hardware |

Not covered by the checker, because it needs a full simulation: interactions with the supervisor (the PID case
above). The supervisor tests are the guard there. The autotuner should include a supervised takeoff (follow-up).

## 10. Controller current and the firmware co-dependency limit

- **Continuous current is not a hard cap.** The moteus r4.11 runs above its 12 A no-heat-sink figure (20–32 A
  quoted with heat sinking, and the drives will be water-cooled). The sim uses `drive_i_cont` (20 A placeholder)
  only for a thermal WARN.
- **A firmware co-dependency limit is planned.** It will couple current, torque, rail current and temperature
  across the three drives, so they run inside a safe operating area. Until its rules exist, the sim stands in for
  it with:
  - the per-drive firmware torque cap (`torque_cap`, 28 N·m);
  - the moteus current and voltage limits inside the servo model;
  - the checker's rail and thermal rows.

  When the firmware rules are defined, model them in `actuation/servo.py` (shared rail budget, I²t per drive)
  and add their consistency rows.

## 11. What has to be designed (the mechanism)

1. **A drive nacelle at the strut–foil junction**, holding both main-flap drives with short stub shafts into the
   flap roots. The full-span flaps still meet at the strut. An 80 mm nacelle costs about 16 N of drag at
   8 m/s, about 8 % of cruise thrust.
2. **A sealed wet housing**: oil-filled and pressure-compensated with lip seals, a magnetic coupling on the motor
   side, or a wet-running motor with a sealed gearbox. Water cooling helps the continuous current (section 10).
3. **The balanced flap**: 25 % chord, full half-span, hinge set back 15 %, sealed cove gap, closed torsion box
   driven at the root. Bench or CFD check the set-back before committing.
4. **An absolute encoder at each surface root, with dual-loop feedback**: position from the surface encoder,
   damping from the motor encoder. Output-only feedback rang with 72 N·m through the gear.
5. **A low-backlash or preloaded gear stage.** The balanced flap carries almost no steady hinge load, so the gear
   knocks across its backlash twice per wave without a preload.
6. **Torque limiting and stops**: firmware cap (with the co-dependency limit), hard stops at ±20–25° sized for
   the full drive torque, a slip clutch for strikes, and an output shaft of at least 13 mm.
7. **Fail-safe behaviour (doc 19)**: a spring return or brake on the elevator. A balanced flap floats less when
   limp.
8. **Command shaping**: the 150°/s slew limit (now in the sim). Carry it into the flight software.

## 12. Limits of this study

- **Effectiveness scaling is first order** (constant loss factor 0.56). The roll arm does not move with the flap's
  spanwise position, and the elevator's twist and the flap's twist are uniform-load approximations.
- **Not modelled:** plain-flap separation above 15–20°, and the cove gap. The set-back benefit is a thin-theory
  upper bound.
- **Placeholders:** backlash, gear stiffness, GJ, flap inertia, `drive_i_cont`, gearbox rating, shaft material.
  The sim has the effects; the values need the chosen parts.
- **Not modelled in the drive:** gear tooth impact loads (knocking is counted, not its force), and flap torsional
  dynamics (twist is quasi-static).
- **The LQR's drive model** is the rigid single mass. Backlash, compliance and twist are robustness, not design.
- **Hand estimates:** the nacelle drag and the transmission numbers.
