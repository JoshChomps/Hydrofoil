# 06 — Control System

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

## Sign conventions (from `pid.py`)

| Loop | Error | Meaning |
|---|---|---|
| heave | `z_meas − z_ref` | positive = too deep → needs more lift (positive δ → more AoA → lift up) |
| pitch | `θ_meas − θ_ref` | positive = nose up → positive δ_elev → more up-lift at tail → nose down |
| roll | `φ_meas − φ_ref` | positive = rolled right (starboard down) → left foil gets `δ_main − δ_roll`, right gets `+δ_roll` |
| surge | `speed_ref − u` | conventional |

φ positive = starboard down; Unity A/D = bank left/right = negative/positive `phi_ref`.

## PID implementation (`control/pid.py::PID`)

```
integral += e·dt
u = Kp·e + Ki·integral + Kd·(meas − meas_prev)/dt     # derivative on MEASUREMENT
u_clamped = clip(u, min, max)
if u != u_clamped: integral −= e·dt                    # conditional anti-windup (undo this step)
```

- Derivative on measurement avoids "derivative kick" on setpoint jumps. It assumes `error` and `measurement`
  move in the same direction — true for heave (`e = z − z_ref`), pitch and roll. For **surge**, `e = ref − u`
  moves *opposite* to `u`, so the derivative term has the opposite sign from a standard PID (Kd is 0 by default, so this is latent).
- First call after reset has no derivative (no spurious kick).
- No derivative filter, no setpoint weighting, no output rate limit (the actuator provides that).

## Actuator model (`control/actuator.py`)

Per channel: `delayed = buffer[0]` (FIFO of `delay_steps`) → `target = pos + clip(delayed − pos, ±rate·dt)` →
`pos += dt/(τ+dt) · (target − pos)`. The returned angle is what the plant sees (and what Unity animates).
Modeled after a MyActuator RMD-X8-class direct-drive BLDC over CAN; numbers are placeholders.

## Gains and rationale

> **Units warning (doc 13):** the roll and pitch PIDs receive error in **radians** and output **degrees**, so `Kp_roll = 3.5` is 3.5 °/rad ≈ 0.061 °/° — about 57× weaker than the numbers suggest. Heave (m → °) is fine.

| Loop | Kp / Ki / Kd | Notes |
|---|---|---|
| heave | 15 / 1 / 5 | Higher than the legacy `sim.py` (1/0/3) because the vehicle starts at rest and must generate takeoff lift; only loop with nonzero Ki |
| pitch | 2 / 0 / 2 | |
| roll | 3.5 / **0** / 2.5 | Ki must remain 0 |
| speed | 30 / 5 / 0 | Output thrust N, ±50 |

**Not yet retuned** after flap effectiveness (0.21), finite-span, free-surface, induced-drag and added-mass corrections were introduced.

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
