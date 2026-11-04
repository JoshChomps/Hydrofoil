# 10 — Roadmap and Future Development

Source tags: **[stated]** = in upstream README / `PROJECT_STATE.md`; **[suggested]** = my recommendation from reading the code.

## Phase 0 — Fix the baseline (from doc 13)
1. Fix the rad/deg unit mismatch in the roll/pitch PIDs (convert error to degrees or scale gains ×57.3), retune, and commit the bank-step and disturbance-recovery tests (the sweeps in doc 05's re-test table are the acceptance criteria).
2. Speed-scheduled feed-forward trim δ_trim ≈ W/(2·q·A·CLα·k_flap) so height settles in seconds; raise default `t_end`.
3. Make waves able to excite roll/yaw (per-foil phase, directional spread, lateral components); lock λ–T by dispersion.
4. Add strut side force and a buoyancy lever arm (roll/yaw stability), Munk moment, wing–tail downwash, section pitching moment of the flap.
5. Replace the freeze-style anti-windup with conditional integration or back-calculation; add a derivative filter; fix the surge Kd sign.
6. Move the safety clamps out of the design plant; consider quaternion attitude.

## Phase 3 — Advanced control (stated roadmap)
1. **[stated]** Linearised state-space model (A, B) about a foilborne trim point. Nothing exists yet.
   - **[suggested]** Add `hydrofoil/linearize.py` with a trim solver (solve `physics_rhs = 0` for δ_main, δ_elev, thrust at given u, flying_height) and a central-difference Jacobian; add tests that A has the expected open-loop poles/stability.
2. **[stated]** LQR / MPC as a coupled MIMO controller to replace the three SISO PIDs; expected to cure the combined-input divergence (doc 05 §2.4).
3. **[suggested]** Gain/trim scheduling in speed (lift ∝ V²) and a feed-forward trim flap angle.
4. **[suggested]** Retune the existing PIDs after the foil corrections as a baseline to beat; record step-response metrics so LQR/MPC can be compared.

## State estimation and sensors (stated, not started)
- `sensors/`: IMU (gyro/accel noise, bias), depth/height sensor (ultrasonic/pressure with latency and dropout), GPS/speed.
- `estimation/`: complementary/Kalman filter (EKF) from sensor outputs; feed estimator output to the controllers instead of ground truth.
- **[suggested]** Add a `--noise` option so every controller test can run in both ideal and realistic modes.

## Model fidelity (suggested)
- Measured parameters: mass, inertia, CG, foil geometry/positions, hull waterplane/drag, actuator dynamics, thrust curve.
- Replace placeholder `free_surface_lift_factor` with validated data (panel method / tow tank).
- Per-surface polars at the right Reynolds number (Re ≈ 6e5 vs 2.7e6 now); add post-stall, negative-α table symmetry (table stops at −10° but +17.8°).
- Foil mounting incidence (`aoa_offset`) so body pitch influences foil AoA; strut drag and ventilation limits.
- Full added-mass/damping matrices (surge, sway, roll, yaw) and added-mass Coriolis.
- Real hull-drag curve with the pre-takeoff hump; hull trim/heel restoring moments (buoyancy centre not at the body origin).
- Irregular seas (JONSWAP / Pierson–Moskowitz), wave direction control, currents, finite-depth dispersion; compute orbital velocity at each foil rather than at the origin.
- Twin-motor thrust model with differential thrust and propeller/motor dynamics (matches the stated hardware: two Maytech MTSPF75200 VESC ESCs, NUCLEO-H723ZG).
- Freshwater density option (lake testing).

## Software engineering (suggested)
- (done) `DEFAULT_PARAMS` moved to `hydrofoil/params.py`.
- Add `pyproject.toml` / pinned versions; add `plotly` and `pytest` to requirements; document Python ≥ 3.10.
- Tests: `vehicle.py` (trim, takeoff reaches foilborne, finite after fuzz), PID (anti-windup, derivative-on-measurement), hull, waves, free-surface factor, uplink clamps.
- Headless regression scripts (the bank sweep, fuzz test, Ki divergence check) checked in under `tests/` or `scripts/` — they currently exist only as ad-hoc throwaways.
- Retire or rebuild `sim.py` on top of `vehicle.py`.
- Add a CSV/NPZ run logger for analysis; fill the (non-existent) `notebooks/` directory.
- Real-time pacing: catch-up logic or fixed-rate scheduler; run physics in a separate process from the dashboard UI.

## Viewer (suggested)
- Done: Unity removed, MuJoCo viewer added (doc 08).
- Next: wave-surface mesh, spray/flow cues, trail of the track, offline video rendering, sharing live params with the dashboard.

## Hardware track (stated: deprioritised)
Firmware for NUCLEO-H723ZG, VESC CAN control, MyActuator RMD-X8 CAN position control; hardware-in-the-loop with the sim; then replace the actuator and thrust placeholders with bench numbers.

## Suggested ordering
1. Stabilise baseline: fix doc/requirements gaps, add `vehicle` tests, check in the stability sweeps. 2. Retune PIDs on current physics. 3. Trim + linearisation. 4. LQR. 5. Sensors + estimator. 6. MPC and hardware.
