# 20 — Main-foil flap and drive design (manned preset)

How the main foil's flap geometry and the drive's gear ratio set the hinge torque, motor current and
mechanical loads. Which mechanical constraints follow from that, and what mechanism still has to be
designed. All numbers are for `MANNED_PARAMS` (275 kg, 0.28 m chord × 1.1 m per half, 3° incidence,
8 m/s cruise) and the mj5208 + moteus r4.11 drive at 40 V unless noted.

Reproduce:

    python tools/flap_design.py static                                 # sections 2, 3 and 4 (about 3 s)
    python tools/flap_design.py fly --cf 0.25 --sf 1.0 --sb 0.2 --N 20  # one row of section 5 (a few minutes)

## 1. What changed in the model

- **Flap lift effectiveness now follows the geometry.** It used to be a constant, 0.21, whatever the flap
  chord or span (`forces/foil.py`), while the hinge moment did follow the geometry. A geometry sweep was
  therefore inconsistent: a smaller flap had a smaller hinge moment but the same lift.
  `main_flap_effectiveness(c_f/c, span_frac)` scales the 0.21 by Glauert's τ(c_f/c) and the flapped span
  fraction. It returns exactly 0.21 at the 40 % chord / half-span reference, so the defaults and both presets
  are unchanged (test). `params["main_flap_effectiveness"]` overrides it.
- **Set-back hinge (overhang balance).** The hinge axis can sit `main_flap_setback` × c_f aft of the flap's
  leading edge (`hinge.thin_airfoil_coeffs(..., setback=)`, `SurfaceGeometry.hinge_setback`). Lift is
  unchanged. The load on the overhang ahead of the axis opposes the rest. Default 0, unchanged.
- **New tool `tools/flap_design.py`.** It sweeps geometry and gear ratio. In the closed-loop mode it
  re-derives the trim feed-forward and the LQR for each geometry.

## 2. Flap geometry: the trade

Thin-airfoil coefficients (per rad; the hinge moment is referenced to q·S_f·c_f; the sim applies a 0.8
viscous factor):

| c_f/c | set-back | τ (lift per flap deg) | Ch_α | Ch_δ | hinge torque for the same lift change (40 % plain = 1) |
|---|---|---|---|---|---|
| 0.20 | 0 | 0.55 | −0.50 | −0.93 | 0.31 |
| 0.25 | 0 | 0.61 | −0.57 | −0.95 | 0.45 |
| 0.30 | 0 | 0.66 | −0.63 | −0.97 | 0.61 |
| 0.30 | 0.1 | 0.66 | −0.47 | −0.68 | 0.43 |
| 0.30 | 0.2 | 0.66 | −0.30 | −0.39 | 0.24 |
| 0.30 | 0.3 | 0.66 | −0.14 | −0.10 | 0.06 |
| **0.40** (current) | 0 | 0.75 | −0.75 | −1.01 | **1.00** |
| 0.50 | 0 | 0.82 | −0.86 | −1.07 | 1.50 |

What sets the hinge torque:

- **Hinge torque for a given lift change goes as c_f²·Ch_δ/τ, and the flapped span cancels.** A longer
  flap has more area but needs proportionally less deflection. So the chord fraction sets the torque.
  A 20–25 % flap needs a third to a half of the current 40 % flap's torque.
- **Span sets the deflection.** A smaller chord means more deflection for the same lift, because τ falls.
  The flapped span fraction divides the deflection directly. A plain flap loses effectiveness past about
  15–20° (separation; **not modelled**: the sim's flap stops are at 30° and its lift is linear in δ).
  So a small-chord flap has to be long.
- **A set-back hinge is the strongest lever.** A 20 % set-back cuts Ch_δ by about 60 % and Ch_α by about
  half. In thin theory the flap overbalances (the moment reverses and the flap runs away) at about 33 %
  set-back. Real cove flow gives less benefit than thin theory, so **15–20 % is the practical range**; treat
  the table as an upper bound.

Per main-flap drive at level trim, with ±3° of effective angle-of-attack authority at 30°/s demanded of
every geometry. M_pk is the trim moment plus the authority moment; I_hold is the motor current that holds
the trim moment at 20:1.

| c_f/c, span, set-back | speed (m/s) | effectiveness | trim flap | deflection needed | trim moment (N·m) | M_pk (N·m) | I_hold at 20:1 (A) |
|---|---|---|---|---|---|---|---|
| 0.40, 0.5, 0 (current) | 6.0 | 0.21 | 6.6° | 20.9° | −8.4 | 22.2 | **17.2** |
| | 8.0 | | −2.8° | 17.1° | 1.0 | 25.4 | 2.0 |
| | 9.5 | | −6.3° | 20.6° | 9.9 | **44.3** | **20.1** |
| 0.25, 0.5, 0 | 9.5 | 0.17 | −7.9° | **25.4°** | 5.4 | 20.8 | 10.9 |
| 0.25, 1.0, 0 | 9.5 | 0.34 | −3.9° | 12.7° | 3.8 | 19.2 | 7.7 |
| 0.40, 0.5, 0.2 | 9.5 | 0.21 | −6.3° | 20.6° | 3.6 | 17.6 | 7.4 |
| **0.25, 1.0, 0.2** | 6.0 / 8.0 / 9.5 | 0.34 | 4.1° / −1.7° / −3.9° | 12.9° max | −1.8 / −0.2 / 1.2 | 4.2 / 4.6 / **7.4** | 3.6 / 0.4 / 2.5 |

Full table: `tools/flap_design.py static`.

**The current flap does not fit the drive.**
- At 9.5 m/s the full authority needs 44 N·m, against the 28.9 N·m the mj5208 × 20:1 can give.
- Holding trim at the 6 m/s takeoff speed needs 17 A continuous, against the moteus's 12 A rating without a
  heat sink. At 9.5 m/s it needs 20 A.
- The 25 %-chord, full-span, 20 % set-back flap needs 7.4 N·m at top speed and at most 3.6 A to hold trim.
  It also keeps the deflection under 13°, inside the plain-flap linear range.

## 3. Gear ratio

| N | full output torque (N·m) | reflected rotor inertia (kg·m²) | total at the flap (kg·m²) | servo natural frequency, kp 250 (Hz) |
|---|---|---|---|---|
| 10 | 14.4 | 0.007 | 0.036 | 13.3 |
| 14 | 20.2 | 0.014 | 0.042 | 12.2 |
| 20 | 28.9 | 0.028 | 0.057 | 10.6 |
| 30 | 43.4 | 0.063 | 0.092 | 8.3 |
| 50 | 72 | 0.175 | 0.204 | 5.6 |

The total inertia is structure 0.009 + added water mass 0.020 + reflected rotor.

- **Motor speed never sets the gear ratio.** The authority demands, 70–195°/s of flap, leave the back-EMF limit
  at N ≈ 240–660. Even the takeoff command steps (about 600°/s) stay below it.
- **The lower bound is torque**: N ≥ 1.5 × M_pk / (1.7 N·m × 0.85). For the current flap that is about 46 at
  9.5 m/s. For the 25 % / full-span / 20 % set-back flap it is about 8.
- **The upper bound is reflected inertia and back-drivability.** The rotor's reflected inertia grows as N²
  and passes the surface's own inertia (structure plus water) near N = 20. Beyond that the servo slows, and a
  limp drive (fault, power loss) is harder for the water to back-drive. That matters for the fail-safe the
  supervisor relies on (doc 19).
- **Current falls as 1/N** (closed loop, head sea, current flap). From 14:1 to 20:1 to 30:1 the rms current
  goes 14.1 → 9.8 → 6.4 A. At 14:1 it is over the 12 A continuous rating.

**Recommendation: keep 20:1 (14–20:1 is fine with the balanced flap).** With the smaller balanced flap,
torque has a 3–4× margin at 20:1, and 10:1 also works (section 5). Choose N for the jam, stop and inertia
loads below, not for speed.

## 4. Mechanical constraints

The drive can always put its full output torque into a stop, a jam or a weed strike. Size the load path for
that, or limit it.

| N | full torque (N·m) | solid output shaft, 316 SS, SF 2 on shear yield | pushrod force at a 20 / 30 / 40 mm horn |
|---|---|---|---|
| 10 | 14.4 | ≥ 10.2 mm | 720 / 480 / 360 N |
| 20 | 28.9 | **≥ 12.8 mm** (the 8 mm placeholder in `DriveSpec` is overstressed 4×) | 1450 / 960 / 720 N |
| 30 | 43.4 | ≥ 14.7 mm | 2170 / 1450 / 1080 N |

### Room inside the foil
The polar is NACA 0009, 0.28 m chord, 25 mm maximum thickness. At the hinge line the section is:

| hinge position | section thickness |
|---|---|
| 60 % chord (40 % flap) | 19 mm |
| 70 % chord | 15 mm |
| 75 % chord (25 % flap) | 13 mm |

- **A torque-carrying shaft of 12–13 mm does not fit along the hinge line.** The torque has to go in at the
  flap's root. The spanwise hinge then only carries shear: small pins, or a continuous or flexure hinge.
- **The flap itself becomes a torsion member.** A 1.1 m carbon flap with a closed section (about 1.5 mm
  wall, G ≈ 20 GPa) twists about M·L/(2GJ) ≈ 1° at 8 N·m. At the balanced flap's 2 N·m it twists about
  0.25°. This is another reason to balance the flap.
- **The set-back needs room for its nose.** A 20 % overhang of a 70 mm flap is 14 mm ahead of the axis, in a
  cove of the 13–15 mm section. That fits, but the cove gap must be sealed: hinge-gap leakage is part of why
  the effectiveness is only about 55 % of τ × span.

### Getting the torque to the flap root (1.65 m main strut)
Steel options, stiffness at the flap compared with the servo's 250 N·m/rad (they act in series):

| transmission | stiffness at the flap (N·m/rad) | unguided buckling load | verdict |
|---|---|---|---|
| 8 mm rod, 30 mm horn | 5500 | 146 N | buckles at 1/7 of the 960 N load |
| 16 × 1 mm tube, 30 mm horn | 5100 | 965 N | marginal; needs guides, which also need room |
| 20 × 1.5 mm tube, 30 mm horn | 9500 | 2700 N | works, but the strut is 0.16 m × about 21 mm thick |
| torque tube 8 mm to 20 × 1.5 mm | 19–359 | — | far too soft: comparable with or below the servo kp |

A pushrod or tube also adds backlash and compliance outside the motor's encoder. The delay-aware LQR feeds
back the achieved flap angle, so it would need an encoder on the flap itself.

### Rail current
Rail current is the sum of the three drives' bus power divided by 40 V.
- Cruise in waves: 0.1–1 A mean, at most 2.4 A.
- Takeoff: 17–21 A peak in every configuration. That is close to the 25 A supply, and it comes only from the
  takeoff command step (section 5).
- Propulsion for the manned boat (about 190 N at 8 m/s, about 4.3 kW at 35 % overall efficiency, about
  110 A at 40 V) cannot share that rail. It needs its own battery path.

## 5. Closed loop (LQR, servo drives in the loop, LQR and trim feed-forward re-derived per geometry)

Scenarios:
- **takeoff**: from rest to 8 m/s in calm water, 40 s.
- **head / following**: from a flying start, in a 0.15 m, 10.8 m sea, 20 s.

Main-flap drive values; "rail" is all three drives together.

| c_f/c, span, set-back, N | head sea height rms | head M_pk / I_rms | following M_pk / I_rms | head rail mean / peak | takeoff I_pk / rate_pk / rail peak |
|---|---|---|---|---|---|
| 0.40, 0.5, 0, 20 (current) | 18.2 mm | 7.7 N·m / 9.8 A | 6.9 / 8.0 | 0.60 / 1.3 A | 48 A / 549°/s / 17.6 A |
| 0.40, 0.5, 0, 14 | 18.1 | 7.7 / **14.1** | 6.9 / 11.4 | 1.04 / 2.4 | 59 / 586 / 21.4 |
| 0.40, 0.5, 0, 30 | 18.4 | 7.6 / 6.4 | 6.8 / 5.3 | 0.36 / 0.7 | 32 / 475 / 12.7 |
| 0.30, 0.5, 0, 20 | 17.6 | 5.5 / 6.1 | 5.1 / 5.4 | 0.36 / 0.7 | 48 / 605 / 17.8 |
| 0.25, 0.5, 0, 20 | 17.5 | 4.4 / 4.5 | 4.1 / 4.1 | 0.29 / 0.5 | 48 / 609 / 16.9 |
| 0.25, 1.0, 0, 20 | 15.0 | 4.6 / 5.2 | 4.0 / 4.4 | 0.29 / 0.5 | 48 / 604 / 17.8 |
| 0.40, 0.5, 0.2, 20 | 17.9 | 3.4 / 6.6 | 2.9 / 3.8 | 0.41 / 0.7 | 48 / 549 / 17.6 |
| 0.25, 0.75, 0.2, 20 | 15.8 | 1.7 / 2.7 | 1.5 / 1.8 | 0.23 / 0.3 | 48 / 607 / 17.4 |
| **0.25, 1.0, 0.2, 20** | **14.9** | **2.0 / 2.9** | **1.7 / 1.9** | **0.22 / 0.3** | 48 / 604 / 17.8 |
| 0.25, 1.0, 0.2, 10 | 14.7 | 2.0 / 6.0 | 1.7 / 3.9 | 0.45 / 0.8 | 59 / 585 / 18.5 |

None failed (no touchdown after liftoff, no roll beyond 30°). The flap angle range in waves is about −5° to
+3° on the full-span flaps and −9° to +4° on the half-span 25 % flap. Every takeoff reaches the 20° command
limit.

- **In waves the balanced, smaller, longer flap is better on every count**: 4× lower hinge moment, 3× lower
  motor current, 3× lower rail current, and 3 mm less height error. It has more lift per degree, so it does
  the same job with smaller and slower flap motion.
- **The takeoff peaks are the same for every geometry.** The takeoff loop steps the flap to its 20° command
  limit at 550–610°/s. The drive saturates its torque on the acceleration (23.8 N·m and 48 A at 20:1). It is
  the command, not the water: hinge moment is under 14 N·m in all of these runs. A slew limit of about
  100–150°/s on the takeoff flap command would remove these peaks and the 17–21 A rail transient. **Not yet
  done.**
- The LQR's input weights are in flap degrees. A less effective geometry is therefore penalised slightly more
  per unit of control. That is a fair comparison of hardware, not of tuning.

## 6. What has to be designed (the mechanism)

There is no off-the-shelf part that does this. The pieces to design, in order of risk:

1. **A drive nacelle at the strut–foil junction.** The flaps are on the inboard half of each half-span, so
   both flap roots meet at the strut. One streamlined nacelle there can hold both main-flap drives (motor,
   gearbox, output encoder) with short stub shafts into the flap roots. That avoids a 1.65 m transmission
   (section 4: a pushrod buckles and a torque tube is too soft). An 80 mm nacelle costs about 16 N of drag
   at 8 m/s, about 8 % of cruise thrust. With full-span flaps (recommended) the roots are still at the strut.
   The alternative, motors in the hull with guided 20 mm push-pull tubes and bellcranks at the junction,
   fits the strut only just and adds backlash.
2. **A sealed wet housing.** Pick one of:
   - an oil-filled, pressure-compensated housing with lip seals on the output shafts (ROV practice);
   - a magnetic coupling through a sealed wall on the motor side (1.7 N·m is small enough), with no dynamic
     seal at all;
   - a wet-running motor (potted stator, stainless or ceramic bearings) driving a sealed gearbox.

   The water cools the motor, which is useful because the holding current is the thermal limit (section 2).
3. **A balanced flap: 25 % chord, full half-span, hinge set back 15–20 % of the flap chord**, with a sealed
   cove gap. The hinge line carries shear only. The flap is a closed torsion box driven at its root.
   Confirm the set-back on a bench or in CFD before committing: overbalance makes the flap diverge.
4. **An output-side absolute encoder on each surface.** It sees backlash and compliance that the motor
   encoder cannot. The LQR feeds back the achieved angle and rate. Planetary backlash of 10–20 arcmin is
   0.2–0.3°, enough for an integral loop to limit-cycle. Either preload it out or choose a low-backlash
   (cycloidal) stage.
5. **Torque limiting and stops.** Set the firmware cap near 1.5× the real hinge peak: about 10–12 N·m with
   the balanced flap, not 28. Hard stops at ±20–25° must take the full drive torque. A slip clutch protects
   against weed and grounding strikes.
6. **Fail-safe behaviour (doc 19).**
   - **Elevator:** a spring return or brake to 0°.
   - **Main flaps:** a limp plain flap floats trailing-edge up to δ = −(Ch_α/Ch_δ)·α, which loses lift (the
     boat settles). Asymmetric floating rolls the boat. A balanced flap floats less and back-drives more
     easily, so the supervisor's jammed-flap reconfiguration has a better starting point.
7. **Command shaping**: a slew limit on the takeoff flap command (section 5). This is software, but it sets
   the drive and supply peaks.

## 7. Limits of this study

- Effectiveness scaling is first order. It keeps a constant loss factor (0.21 / (τ·span) ≈ 0.55), and the
  roll arm does not move with the flap's spanwise position.
- Plain-flap separation above about 15–20° is not modelled, so large-deflection small flaps look better than
  they are.
- The set-back benefit is thin-airfoil theory, an upper bound.
- Gearbox backlash, flap torsional compliance and linkage compliance are not in the servo model.
- Unsteady hinge moments use quasi-steady Theodorsen.
- The mechanical numbers (shaft, rods, flap twist, nacelle drag) are hand estimates with placeholder
  materials. They are for sizing, not certification.
- Only the main flaps were studied in depth. The all-moving elevator draws 8–9 A peak in head seas and
  about 2–4 A in following seas at 20:1. Its pivot (0.22 c, just ahead of the aerodynamic centre) already
  balances it.
