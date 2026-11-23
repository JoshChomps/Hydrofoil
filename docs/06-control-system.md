# 06 — Control System

> **Update Notice:** The current flight control stack combines takeoff/landing SISO PIDs (`control/pid.py`) with speed-scheduled delay-augmented MIMO LQR (`control/lqr.py`), `auto` handover blending, `Supervisor` flight modes and active sideslip CBF unwinding (`control/supervisor.py`), and a physics-constrained 3-layer Jacobian QP `ControlAllocator` (`control/allocator.py`) with LQR anti-windup rollback (`sync_sent(..., sat_flags=...)`). See [00-master-system-architecture.md](00-master-system-architecture.md), [19-linearisation-lqr-estimation.md](19-linearisation-lqr-estimation.md), and [24-dual-front-flaps-and-jacobian-allocation.md](24-dual-front-flaps-and-jacobian-allocation.md). The sections below preserve the baseline SISO PID specification and sign conventions (which still run during displacement/takeoff).

## Architecture

```
          pilot / UI setpoints                                   plant (vehicle.py)
 flying_height ─► z_ref = hull_offset − flying_height ─┐
 phi_ref [deg] ─► rad ─────────────────────────────────┤  heave PID ─► cmd_main ─► Actuator ─► δ_main  ─┐ left  = δ_main − δ_roll
 theta_ref [deg] ─► rad ───────────────────────────────┤  pitch PID ─► cmd_elev ─► Actuator ─► δ_elev  ─┤ right = δ_main + δ_roll
 target_speed ─► surge PID ───────────────────────────►│  roll  PID ─► cmd_roll ─► Actuator ─► δ_roll  ─┘ elev  = δ_elev
                                                        └ thrust [N] ─► +x force
 feedback: exact η, ν from the simulator (no sensors, no estimator)
```

Three independent SISO loops plus one speed loop. No decoupling, no feed-forward, no gain scheduling, no MIMO.
**Note:** the code and older docs call this a "cascade PID"; it is not — there is no inner rate loop under an outer angle loop. Each loop is a single parallel-form PID acting directly on an actuator command.

## Sign conventions (from `pid.py`)

| Loop | Error | Meaning |
|---|---|---|
| heave | `z_meas − z_ref` | positive = too deep → needs more lift (positive δ → more AoA → lift up) |
| pitch | `θ_meas − θ_ref` | positive = nose up → positive δ_elev → more up-lift at tail → nose down |
| roll | `φ_meas − φ_ref` | positive = rolled right (starboard down) → left foil gets `δ_main − δ_roll`, right gets `+δ_roll` |
| surge | `speed_ref − u` | conventional |

φ positive = starboard down; viewer Left/Right = bank left/right = negative/positive `phi_ref`.

## PID implementation (`control/pid.py::PID`)

```
candidate = I + e·dt
u = Kp·e + Ki·candidate + Kd·(y − y_prev)/dt         # derivative on MEASUREMENT (PI-D), no filter
u_sat = clip(u, lo, hi)
pushing_deeper = (u > hi and e > 0) or (u < lo and e < 0)
if not pushing_deeper: I = candidate                  # conditional integration (fixed, doc 14 E3)
```
- First call after `reset()` has no derivative. Derivative-on-measurement avoids setpoint kick.
- Angle loops: `HydrofoilController.update` takes angles in **radians** and converts errors/measurements to **degrees** before the PID, so Kp/Kd are ° of flap per ° (per °/s) (fixed, E1).
- Surge: the measurement fed to the PID is `−u` so Kd opposes acceleration (fixed, E4).

## Actuator models (`params["actuator_model"]`)

**`"servo"` (default since 2026-11-05)**, `actuation/servo.py`, doc 17 §7. The PIDs' raw virtual commands are **mixed first**
(left = main − roll, right = main + roll, elevator). Each physical surface then has its own drive: a transport delay of 10 ms,
moteus position PD (`kp` 250 N m/rad, `kd` 4.5 N m s/rad at the output), the firmware torque cap (10 N m), current and back-EMF
voltage limits, gearbox efficiency, rotor + water + structure inertia, hydrodynamic damping, and the steady hinge moment at the
surface's own flow. All of this is integrated in 1 ms sub-steps against mechanical stops (flaps ±30°, elevator ±20°). Slew rate,
lag and saturation come out of the physics. The plant sees the drives' actual angles. Optional `q_schedule_speed` scales the flap
commands by (v/u)² above that speed.

**`"kinematic"` (legacy)**, `control/actuator.py`, described below. It acts on the virtual channels before mixing.

Per channel: `delayed = buffer[0]` (FIFO of `delay_steps`) → `target = pos + clip(delayed − pos, ±rate·dt)` →
`pos += dt/(τ+dt) · (target − pos)`. The returned angle is what the plant sees (and what the viewer animates).
Modeled after a MyActuator RMD-X8-class direct-drive BLDC over CAN; numbers are placeholders.

## Gains and rationale (retuned 2026-11-04)

| Loop | Kp | Ki | Kd | Notes |
|---|---|---|---|---|
| heave | 40 °/m | 6 | 10 | Takeoff 2.9 s, 2 cm settling ≈ 17 s, no overshoot (Ki 15 overshoots; Kp 80 is slower). Integrator still carries the ≈ 14° trim flap |
| pitch | 1.0 °/° | 0 | 0.1 | Kd > 0.3 only adds flap chatter |
| roll | 2.0 °/° | **0** | 0.2 | Kd ≥ 1 chatters; Kp ≥ 3.5 uses 13° of flap for a 5° kick |
| speed | 30 N/(m/s) | 5 | 0 | cruise thrust ≈ 17 N of 50 N |

Procedure: closed-loop sweeps on the corrected plant from a clean settled state (metrics: ISE, residual oscillation, flap total variation and peak, takeoff time, settling, overshoot, bank tracking, wave response, delay/lag-doubling robustness). Details and the earlier contamination pitfall: doc 14 E2.
The old values (3.5/2.5 roll, 2/2 pitch, 15/1/5 heave) were tuned for a different plant and, once the units were fixed, limit-cycled (pitch) or chattered (roll).

## Method summary and what changed (full detail: doc 13 §3, doc 14)
- Algorithm: positional discrete PID, rectangular integral, backward-difference derivative **on measurement** (PI-D), no derivative filter, clamp ±20°, **conditional-integration anti-windup** (fixed).
- Execution: sample time = integration step (10 ms), ZOH across RK4 stages, then delay (0.02 s) → rate limit (720 °/s) → lag (20 ms).
- Fixed: rad→deg unit mismatch, surge Kd sign, anti-windup rule, delay-in-steps, gains retuned.
- Still open: no trim feed-forward (integrator builds the ≈ 15° trim), no speed scheduling (lift ∝ V²), shared main-flap authority between heave and roll, no reference shaping, no derivative filter, exact-state feedback, no formal stability margins.

## Pilot interface

Pilot input is never raw surface or force control: it moves **setpoints** of the PIDs (height → heave PID,
throttle → speed PID, bank → roll PID), i.e. a stick into an autopilot. Pitch target is fixed by `theta_ref` (dashboard slider, restart required).

## Known control limitations
- Single-axis loops fight gyroscopic coupling; large or combined inputs can diverge (doc 05).
- Heave and roll share the main-flap channel (symmetric vs differential) and flap saturation at ±20° is shared in effect.
- Heave loop commands the main flap with only 0.21 effectiveness → authority-limited at low speed.
- No speed scheduling: lift ∝ V², so loop gain changes ~16× between 1 and 4 m/s.
- No yaw / heading control; turns arise only from banked flight.

## Starting points for the planned MIMO work (see roadmap)
1. Linearise `physics_rhs` about a foilborne trim (u≈4 m/s, hull clear of water, δ's at trim) by finite difference to obtain A, B (12×12, 12×3 with inputs δ_main, δ_elev, δ_roll plus thrust).
2. Remove the non-smooth pieces for linearisation: table clamp, hull draft clip, and the speed cut-off do not matter near trim, but the Euler/`arctan2` terms and the table's piecewise-linear slope should be checked.
3. Include actuator states (lag/delay) in the augmented model, or use Padé for the delay.
4. Design LQR first (check controllability of the heave/pitch/roll subsystem), then add an observer once `sensors/` and `estimation/` exist.
