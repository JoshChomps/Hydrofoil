# 17 — Control-Surface Drives: moteus r4.11 + BLDC + 20:1 gearbox

How hard the water pushes back on the control surfaces, and what that means for motor current, voltage, power,
gear ratio and structural stress. Built on top of the existing loops: the flight controller is unchanged and still commands surface angles.
Since §7 the drives are simulated in the loop (default), so their limits act on the flight.

Code: `hydrofoil/actuation/` (`hinge.py`, `drive.py`, `loads.py`), `tools/actuator_budget.py`, live read-out in the
MuJoCo app (`hydrofoil/viewer.py`). Tests: `tests/test_actuation.py`. Figures refreshed 2026-11-06 after correcting the flap span: the flap covers only the inboard half of each panel (0.3 m, `main_flap_span_frac = 0.5`); the first version wrongly used 0.6 m (doc 18 F3). Hinge loads and currents roughly halved.

## 1. The chain and where the load comes from

```
flight PIDs --(delta_main, delta_roll, delta_elev, virtual)--> mixer --> 3 physical drives
   left flap  = main - roll        right flap = main + roll        elevator
each drive:  moteus r4.11 (FOC current loop + position PD)  ->  BLDC  ->  N:1 gearbox (N = 20 now)  ->  surface
```
The motor has to supply four things at the surface (positive = trailing edge down = more lift):

| Term | Depends on | Formula (output side) |
|---|---|---|
| Steady hinge moment | **boat speed²**, AoA, deflection | flap: `q S_f c_f (C_hα α + C_hδ δ)`; all-moving: `q S c C_L (x_p − x_ac)/c` |
| Hydrodynamic damping | **boat speed × surface rate** | `B = k_B V`, `M = −B δ̇` (quasi-steady Theodorsen) |
| Added inertia of water | surface acceleration | `I_added δ̈` (water dragged along with the surface) |
| Structure, friction, rotor | acceleration, rate sign | `J_struct δ̈`, `τ_f sign δ̇`, and the rotor `J_rotor N² δ̈` |

So yes: the load depends both on how fast the boat goes (q = ½ρV²) and on how fast the surface moves (damping ∝ V δ̇,
inertia ∝ δ̈). Both effects are in the model.

## 2. Hinge-moment physics (`actuation/hinge.py`)

**Steady coefficients** come from 2-D thin-airfoil theory, solved numerically with a 400-panel lumped-vortex model. It is
checked against closed forms: C_l = 2π, AC at c/4, and Glauert's flap effectiveness τ = 1 − (θ_h − sin θ_h)/π
(E = 0.4: 0.7473 vs 0.7478).

| Flap chord E | τ (lift per deflection) | C_hα [/rad] | C_hδ [/rad] |
|---|---|---|---|
| 0.2 | 0.549 | −0.499 | −0.925 |
| 0.3 | 0.660 | −0.627 | −0.967 |
| 0.4 | 0.747 | −0.745 | −1.014 |

Thin-airfoil theory over-predicts hinge moments of real sections, so the code multiplies by `viscous_factor = 0.8` (PLACEHOLDER).
It also multiplies by the same Helmbold finite-span factor and free-surface factor that `forces/foil.py` uses for lift.

**Unsteady terms** use quasi-steady Theodorsen with C(k) = 1. With b = c/2 and pivot `a` half-chords from mid-chord:
`I_added = πρb⁴(1/8 + a²)` and `B = πρb³V(½ − a) − 2πρVb³(a + ½)(½ − a)`. The flap is approximated as an isolated plate hinged
at its own leading edge (a = −1), which ignores the main element ahead of it. Treat its unsteady numbers as order-of-magnitude.

| Drive | Layout | I_added | J_struct (PH) | B per m/s |
|---|---|---|---|---|
| each main flap (0.06 × 0.3 m, inboard half) | plain flap, hinge 60 % c | 0.9e-3 kg m² | 4e-4 | 0.08 N m s/rad |
| elevator (0.10 × 0.3 m) | all-moving, pivot 22 % c | 2.7e-3 kg m² | 4e-4 | 0.14 N m s/rad |
| rotor reflected, 20:1 | — | **2.8e-2 kg m²** | | |

**The motor's own rotor dominates the inertia at 20:1**: J_rotor N² is about 10–20× the surface plus water. The inertia-matching
ratio sqrt(J_load/(η J_rotor)) is about 5 for a flap and 7 for the elevator. That is why the current-optimal ratio below is lower than you might expect.

## 3. Key result: the trimmed hinge moment hardly changes with speed, but control moments grow with V²

Closed-loop cruise in calm water (steady state, left flap):

| Speed [m/s] | Trim flap δ [°] | Trim hinge moment [N m] | Extra moment for +5° flap [N m] |
|---|---|---|---|
| 3 | 20.0 (saturated, not foiling) | −0.86 | −0.30 |
| 4 | 15.1 | −1.04 | −0.53 |
| 5 | 9.8 | −1.05 | −0.83 |
| 6 | 7.0 | −1.08 | −1.20 |
| 7 | 5.4 | −1.13 | −1.63 |
| 8 | 4.4 | −1.18 | −2.12 |

Why this happens: in trim, lift = weight, so q·C_L is fixed. C_L and C_h are both roughly linear in (α, δ), so in trim
α and δ both scale as 1/q. The hinge moment q·S_f·c_f·C_h is then nearly constant, roughly the trimmed lift times a
fixed lever (≈ 0.3 c_f here). A **change** in deflection, though, costs ΔM = q S_f c_f C_hδ Δδ, which grows with V².

What this means:
- **Holding torque** (thermal, continuous current) is set by **weight**, not speed: about 1.0–1.2 N m per main flap.
- **Manoeuvring and gust torque** grows with V². The PID gains are fixed in degrees per metre, so at 8 m/s each degree of flap
  makes 4× the lift (an over-aggressive loop) and needs 4× the torque. **Gain scheduling the flight loop by 1/q fixes both**:
  the commanded Δδ shrinks as 1/V², so actuator torque per disturbance stays roughly constant across speed.
- **Low speed is a lift problem, not a torque problem**: at 3 m/s the flap saturates at the controller's 20° limit and the
  boat cannot take off.

## 4. Budget against the hardware (`tools/actuator_budget.py`)

Hardware assumptions:

| Item | Value | Status |
|---|---|---|
| moteus r4.11 supply | 10–44 V; **40 V** battery → PDB → motors (user-stated 2026-11-10, replaces the 24 V placeholder) | sourced / user |
| PDB output limit | 25 A for the whole rail (`supply_i_max`) | user-stated |
| moteus phase current | 100 A peak; 10–12 A continuous bare, 20–32 A heatsunk | sourced (sources disagree on continuous); the controller runs above 12 A, and a firmware co-dependency limit is planned (docs/20 section 10) |
| moteus peak power | 900 W @ 30 V | sourced |
| Motor | mj5208-like: Kv 330, 1.7 N m peak, 7500 rpm | listing; R_ll 0.10 Ω and J_rotor 7e-5 kg m² are **PLACEHOLDERS** |
| Kt = Ke | 60/(2π Kv) = 0.0289 N m/A | convention, **verify on the bench** |
| Gearbox | 20:1, η 0.85, rated 30 N m, output friction 0.05 N m | **PLACEHOLDERS** |
| Output shaft | Ø 8 mm, 316 SS (shear yield ≈ 140 MPa, SF 2) | **PLACEHOLDER** |

Results at N = 20, V_bus = 24 V (pre-2026-11-10 default; the tool now defaults to 40 V, see doc 12) (worst of left/right; the elevator is in its own row):

| Scenario | Flap δ range [°] | τ_out peak / rms [N m] | Surface rate peak [°/s] | I peak / rms [A] | V needed peak [V] | P_bus peak [W] |
|---|---|---|---|---|---|---|
| takeoff, calm, 4 m/s | 0 … 20 | 1.3 / 1.1 | 240 | 10.8 / 1.9 | 3.0 | 23 |
| cruise 4 m/s, head seas | 0 … 20 | 1.4 / 1.1 | 240 | 10.8 / 1.9 | 3.0 | 23 |
| cruise 4 m/s, beam seas | 0 … 22 | 1.4 / 1.1 | 241 | 10.8 / 1.9 | 3.0 | 23 |
| ±8° bank manoeuvre | −4 … **35** | 3.2 / 1.1 | 240 | 14.5 / 2.1 | 3.5 | 36 |
| cruise 6 m/s, head seas | 0 … 20 | 1.6 / 1.1 | 240 | 10.8 / 2.0 | 3.0 | 23 |
| elevator (all cases) | −2 … 4 | 0.2 / 0.07 | 51 | 3.2 / 0.2 | 0.6 | 3 |

Gear-ratio sweep (worst drive over all scenarios):

| N | I peak [A] | I rms [A] | V peak [V] | P peak [W] | Motor rpm peak | Fails |
|---|---|---|---|---|---|---|
| 5 | 25.7 | 8.1 | 3.2 | 66 | 200 | — |
| 10 | 14.0 | 4.0 | 2.6 | 33 | 401 | — |
| 15 | **13.4** | 2.7 | 3.0 | 32 | 601 | — |
| **20** | 14.5 | **2.1** | 3.5 | 36 | 802 | — |
| 30 | 18.1 | 1.6 | 4.8 | 59 | 1203 | — |
| 40 | 22.4 | 1.4 | 6.1 | 94 | 1603 | — |

What the numbers say:
1. **20:1 works; 10–20:1 is the sweet spot.** Peak current is lowest at about 15:1 and almost flat from 10 to 20:1. Below that,
   holding torque dominates and current goes up. Above it, accelerating the reflected rotor inertia dominates. RMS (thermal)
   current keeps falling with N. Every ratio from 5 to 40:1 passes; 5:1 uses 8 of the 12 A continuous rating.
2. **The controller and motor are lightly used.** About 2 A rms against 12 A continuous, and about 15 A peak against 100 A.
   Voltage needed is about 3.5 V against 21.6 V usable, because the surfaces only move at about 240°/s, roughly 800 rpm at the
   motor against 7500 rpm available. Bandwidth is limited by the outer flight loop and the placeholder 20 ms lag, not by
   the drive.
3. **Electrical power is negligible**: about 2 W per drive average (most of that is board idle) against about 70 W of
   propulsion (17 N × 4 m/s). Regeneration is about 0.1–0.3 J per manoeuvre, so bus pump-up is not an issue on a battery.
   On a bench supply that cannot sink current, still fit bus capacitance or a brake resistor.
4. **The elevator is almost unloaded** because its pivot (22 % c) sits just ahead of the AC (25 % c). Pivot placement is the
   biggest single lever on hinge moment for an all-moving surface. Pivot ahead of the AC weathervanes the surface toward
   zero lift if the drive goes limp (fail-safe). Pivot behind the AC is divergent: the surface runs to its stop. The 3 %
   chord margin assumed here is a PLACEHOLDER. The real AC moves with flap, Reynolds number and free surface, so measure
   it and keep a margin.
5. **Stress: design for what the motor can do, not for what the water does.** The worst hydrodynamic case puts 3.2 N m on
   the output (32 MPa in an 8 mm shaft). At full current the drive can put out 1.7 × 20 × 0.85 ≈ **29 N m**. That is 9× the
   water load, about 290 MPa in that shaft (yields), and right at the
   placeholder gearbox rating. A jam, a hard stop, a rope or a weed strike, or a large position error at high kp will
   apply it. Cap it in firmware: set `servo.max_current_A` or the per-command `maximum_torque` (about 5 N m at the output
   gives 1.6× the worst load; the sim's default `torque_cap` is still 10 N m). Then size the shaft, keys, linkage and flap root for that cap.

## 5. Things this exposed in the existing model

- **The mixing is ahead of the hardware limits.** The PID clamps each virtual channel to ±20°, but left/right = main ∓ roll,
  so in the bank manoeuvre a physical flap reached **35°**. Limits, rate limits and lags belong to the three physical
  drives, after the mixer. The current `Actuator` objects sit on the virtual channels.
- **Flap effectiveness: not a conflict after all** (corrected, doc 18 F2). The original comment says the flap is on the inboard
  half-span. Expected lift effectiveness is ≈ τ 0.75 × ~0.5 lift share × viscous ≈ 0.3–0.36, so 0.21 is low but plausible. If
  it is low, the sim over-deflects by up to ~1.7×, which makes these budgets mildly conservative.
- **The 20° flap saturation at takeoff** shows up in every scenario (δ pinned at 20° during the climb). It is a flight-control
  limit, not a drive limit, but it is the binding constraint on takeoff speed.

## 6. Failure modes to design for

| Event | Effect | Mitigation |
|---|---|---|
| Drive limp or fault (moteus fault, brown-out) | 20:1 planetary is back-drivable, so the plain flap trails to zero hinge moment and lift is lost, and the boat settles onto the hull. All-moving with pivot ahead of the AC does the same. | Acceptable if the pivot is ahead of the AC; keep it that way. |
| Pivot aft of the AC | Divergent: surface slams to its stop and the boat pitches violently | Verify the AC; mechanical stops sized for the full drive torque |
| Current-limited during a gust | Surface lags its command, and the outer loop sees extra delay | Gain-schedule by q; size headroom (≈ 7× peak current) |
| Water ingress / thermal | 2 A rms is easy, but the motor is sealed in a pod | Thermal path to hull/water; use moteus temperature telemetry |
| Over-torque (jam, strike) | Up to 29 N m on the shaft (9× the water load) | Firmware torque cap, shear pin or slip clutch |

## 7. The drive in the loop (done 2026-11-05, now the default)

`params["actuator_model"] = "servo"` (`actuation/servo.py`, wired in `VehiclePhysics._servo_commands`):
1. **The mixer comes first.** The PIDs give raw virtual commands; left = main − roll, right = main + roll, elevator. Limits are now
   **per physical surface**: mechanical stops at ±30° (flaps) and ±20° (elevator), placeholders.
2. **Each drive has state (δ, δ̇)**, integrated in 10 × 1 ms semi-implicit sub-steps per sim step:
   `(J_struct + I_added + J_rotor N²) δ̈ = τ_out + M_hyd(V, α, δ) − B(V) δ̇ − τ_f tanh(δ̇/0.05)`.
   The flow at each surface (relative velocity incl. waves, 3-D and free-surface factors) is frozen over the step, like the wave field.
3. **moteus position mode**: `τ_cmd = kp(δ_cmd − δ) − kd δ̇ (+ optional −M_hyd feed-forward)`, kp 250 N m/rad, kd 4.5 N m s/rad
   at the output (≈ 15 Hz, ζ ≈ 0.8 against J_tot ≈ 0.03 kg m²; PLACEHOLDER tuning). Motor torque is clipped by
   `min(Kt I_peak, motor peak, torque_cap/N)` (cap 10 N m at the output) and by the back-EMF limit `I ≤ (k V_bus − s Ke ω_m)/R`.
   Gearbox: `τ_out = N η τ_m` when driving, `N τ_m/η` when back-driven.
4. **Transport delay 10 ms** (flight-computer loop + CAN-FD), replacing the 20 ms placeholder.
5. **Gain scheduling by q** is available (`q_schedule_speed`: flap commands × (v/u)² above that speed, clipped to [0.25, 1]). It is
   **off** by default; it is not tuned or evaluated yet.
6. The plant sees the drives' **actual** angles; the app shows the drives' own current, power and an AT LIMIT flag.
The legacy kinematic actuator remains available as `actuator_model = "kinematic"`. Cost: 2.3 vs 1.9 ms per step.

**Servo alone** (10° step command): 90 % in ≈ 60 ms after the 10 ms delay. The peak current is 17.3 A, which is the 10 N m firmware
cap doing its job. The steady droop from the hinge moment is 0.16° at 4 m/s and 0.6° at 8 m/s (after the span correction); the outer heave loop's integrator
absorbs it (`feedforward` would remove it).

**Flight with the drives in the loop** (`tools/audit.py` scenarios; the gains were not changed):

| Metric | Kinematic (old) | Servo (default) | Weak servo (kp/4, cap 5 N m, delay 20 ms) |
|---|---|---|---|
| Foilborne at [s] | 2.90 | 2.95 | 3.41 |
| Height within 2 cm [s] | 17.3 | 17.7 | 21.8 |
| Roll 5° kick settles 0.5° [s] | 0.69 | 0.71 | 0.66 |
| 8° bank: final / settle / height loss | 8.46° / 0.55 s / 1.4 cm | 8.47° / 0.56 s / 1.5 cm | 8.53° / 0.55 s / 1.9 cm |
| Pitch 3° kick [s] | 0.56 | 0.57 | 0.54 |
| Height step 0.2 m settles 2 cm [s] | 8.1 | 8.6 | 10.1 |
| Beam / oblique waves max roll [°] | 0.99 / 1.28 | 0.99 / 1.30 | 1.25 / 1.49 |
| Hull touch after takeoff | none | none | none |

The servo column was re-measured after the flap-span correction (2026-11-06); the weak-servo column is from before it (full-span loads, so pessimistic). So the existing gains are robust to real drive dynamics, and a 4× weaker drive degrades gracefully. A retune was not needed.

**Budget flown with the servo** (`python tools/actuator_budget.py --model servo`): the servo moves the surfaces faster than the old
placeholder (peak 467 vs 240°/s). Peak current is ≈ 14 A in every case (the transients are set by rotor inertia, not by water load), rms ≈ 2 A, motor ≤ 1590 rpm,
≤ 5.4 V, ≤ 63 W peak, regen ≈ 1 J. All within limits. Thanks to the per-surface stops the bank manoeuvre's flap now peaks at **29°**
(it was 35°). (The tool's ratio sweep re-analyses trajectories flown at the params' N = 20; for a true sweep, set `gear_ratio`.)

**Failure test** (`tests/test_servo.py`): with the torque cut to zero in cruise, the flaps trail back toward the stream, lift is lost
and the hull returns to the water. That is the expected fail-safe for a plain flap and a pivot-ahead elevator.

Still open: integral action in the drive (moteus has `ki`), a tuned q-schedule, PID anti-windup that knows about **drive**
saturation (the PID only knows its own ±20° clamp), gearbox backlash and compliance, and motor/board thermal state.

## 8. What to measure to replace placeholders

Motor R and Kv (moteus_tool calibration), Kt (torque on a lever plus a scale), rotor inertia (spin-down or datasheet).
Gearbox efficiency both ways and backlash. Surface mass and inertia (CAD). Flap hinge-moment coefficients, or the
elevator AC (tow test with a load cell on the drive). Continuous current with the real pod's cooling.
