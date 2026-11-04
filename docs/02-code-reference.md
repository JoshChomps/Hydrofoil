# 02 — Code Reference (every script)

Repo layout (excluding Unity assets/settings):

```
README.md, requirements.txt, .gitignore
data/foil_tables/naca0009_xflr5_re2700000.dat
hydrofoil/
  vehicle.py  dashboard.py  params.py  viewer.py  sim.py  viz.py  PROJECT_STATE.md   (sim_realtime/stream/control_uplink removed)
  dynamics/   rigid_body.py  kinematics.py
  forces/     foil.py  hull.py  buoyancy.py
  control/    pid.py  surge.py  actuator.py
  waves/      airy.py
  estimation/ (empty)   sensors/ (empty)
tests/        test_forces.py  test_actuator.py   (23 tests, all pass)
```

Data-flow of one physics step (dashboard / realtime):

```
params ─► VehiclePhysics.step(state, t, wave)
            ├─ controller.update(z_ref, z, phi_ref, phi, theta_ref, theta) ─► delta_main/elev/roll (actuator-filtered)
            ├─ surge.update(target_speed, u) ─► thrust
            ├─ wave state (elevation, local surface z, body-frame orbital velocity)
            ├─ RK4 on physics_rhs (controls held constant over the step)
            └─ finite check / clamps ─► new_state, cmds, thrust, draft, wave_elev
```

---

## `hydrofoil/vehicle.py` — canonical physics
- Constants: `MAX_ATTITUDE_RAD = 70°`, `MAX_LINEAR_VEL = 15 m/s`, `MAX_ANGULAR_VEL = 3 rad/s`.
- **`VehiclePhysics(params)`** — builds from a params dict (kept *live*, not copied, so mutating
  `params["flying_height"]` etc. takes effect next step).
  - Computes added mass/inertia (`strip_added_mass_heave_pitch`) from left half, right half, elevator.
  - Creates `RigidBody`, three `Foil`s (left/right main at `y = ∓main_lateral_offset`, elevator on centreline),
    `HydrofoilController`, `SurgeController`. Polar = `FoilTables.naca0009()`.
  - `rest_state()` → 12-vector at rest, z = `hull_offset + min(mass/(ρ·Awp), max_draft)`, pitch 2°.
  - `_wave_state(eta, t, wave)` → (elevation, `water_surface_z = -elevation`, body-frame orbital velocity `Rᵀ·V_ned`).
  - `_submergence_over_chord(foil, eta, surface_z)` → (foil AC world z − surface z)/chord, with AC z = `eta_z + (R·r_attach)_z`.
  - `physics_rhs(state, cmds, thrust, surface_z, wave_vel_body)` → 12-vector derivative (see doc 03).
  - `step(state, t, wave)` → `(new_state, cmds, thrust, draft, wave_elev)`; RK4, safety net.
    Note the returned `draft` is computed from the *pre-step* `eta`.

## `hydrofoil/dashboard.py` (1060 lines) — FastAPI app + embedded HTML/JS
- `DEFAULT_PARAMS` — now imported from `hydrofoil/params.py` (doc 04).
- `SimRunner` — owns params, a thread, a lock and a state dict.
  `start/stop/restart_with(params)/set_live_params(updates)/get_state`. `_run()` loops `vp.step`, publishes
  rounded state, and **sleeps to real time** (`t > elapsed`). Stops at `t_end`.
- Endpoints (doc 07): `GET /`, `GET/POST /params`, `POST /manual`, `/run`, `/stop`, `/reset`, `WS /ws` (30 Hz push).
- `DASHBOARD_HTML` — self-contained page (Plotly 2.32 from CDN):
  - `LIVE_GROUPS` (flight target, propulsion, 3 PID groups, waves) and `CONFIG_GROUPS` (vehicle, hull, foils, sim).
  - Only `flying_height` and `phi_ref` are pushed live (`/manual`, coalesced to ~20 Hz); **everything else needs "Apply & Restart"**.
  - Keyboard stick: W/↑ & S/↓ = height (0.15 m/s), A/← & D/→ = bank (20 °/s, clamp ±10°, spring-back 15 °/s).
  - Canvas views: roll (front), side profile (waves, hull colour by foilborne), depth gauge.
  - Charts: depth, speed+thrust/5, angles, foil deflections; 20 s window, ~10 Hz redraw.
  - `effectiveZRef()` mirrors the backend `z_ref = hull_offset − flying_height`.
- `__main__` block prints local + WSL URLs and runs `uvicorn` on port 8000.

## `hydrofoil/viewer.py` (new) and `hydrofoil/params.py` (new)
See doc 08. `params.py` holds `DEFAULT_PARAMS` (moved out of `dashboard.py`).

## Removed on `mujoco-viewer`: `sim_realtime.py`, `stream.py`, `control_uplink.py`
Kept below for reference (git `841d501`).

## `hydrofoil/sim_realtime.py` (removed)
`run_realtime(speed, streamer, wave, duration, pilot_port)` — builds `VehiclePhysics` from `DEFAULT_PARAMS`,
starts `PilotUplink`, loops: copy pilot values into params → `vp.step` → stream every `int(0.033/dt)` steps
(≈30 Hz at dt=0.01) → print status once per sim-second → sleep to wall clock (`dt/speed`).
CLI: `--speed --host --port --pilot-port --duration(60) --waves` (waves = A 0.05, λ 5, T 2).

## `hydrofoil/stream.py` (removed)
`get_windows_host_ip()` (parses `ip route show default`; falls back to `127.0.0.1`) and `SimStreamer.send(...)`
(JSON UDP, errors swallowed). Packet in doc 07.

## `hydrofoil/control_uplink.py` (removed; limits now in `viewer.py`)
`PilotUplink` — daemon thread bound to `0.0.0.0:5006`; thread-safe properties `flying_height`, `target_speed`,
`phi_ref`; any subset of keys accepted; values clamped on receipt (`HEIGHT -0.3..1.0`, `SPEED 0..10`, `BANK ±8`).

## `hydrofoil/sim.py` — legacy batch sim
Self-contained simplified model (constant-volume neutral buoyancy, speed **pinned** to `CRUISE_SPEED = 4` via
`nu_with_speed[0]`, no wave coupling, foil `flap_effectiveness=1`, no free-surface/hull). 20 s, dt 0.01, start
z=0.3 vs ref 0.5, 2° pitch. Runs closed-loop and open-loop, writes PNGs/HTML to `data/`. Heave gains differ
(`Kp 1, Ki 0, Kd 3`). Requires `plotly` (not in `requirements.txt`). Drifted from `vehicle.py` by design.

## `hydrofoil/viz.py`
`dashboard(log, title, z_ref)` (6-panel Plotly report) and `animation_2d(log, ...)` (frame-based side view),
both write HTML to `data/` and open it via `_open()` (tries `wslpath`+`explorer.exe`, then `xdg-open`, then `webbrowser`).
Only used by `sim.py`.

## `hydrofoil/dynamics/`
- `kinematics.py` — `Rx, Ry, Rz`, `R_body_to_ned = Rz·Ry·Rx`, `T_euler(phi, theta)` (raises `ValueError` if |cosθ|<1e-6),
  `J(eta)` = blockdiag(R, T).
- `rigid_body.py` — `skew`, `strip_added_mass_heave_pitch(strips, rho)`, `RigidBody` (`M`, `C(nu)`, `nu_dot(nu, tau)` via `np.linalg.solve`).

## `hydrofoil/forces/`
- `foil.py` — constants `MAIN_FOIL_FLAP_EFFECTIVENESS=0.21`, `ELEVATOR_FLAP_EFFECTIVENESS=1.0`,
  `MAIN_FOIL_OSWALD_EFFICIENCY=0.80`; `FoilGeometry`, `FoilTables` (`cl`, `cd`, `cl_slope_per_deg`, `from_dat`, `naca0009`),
  `free_surface_lift_factor(h/c)`, `Foil.generalized_force(nu, delta, submergence_over_chord)`.
- `hull.py` — `hull_draft`, `hull_buoyancy_force`, `hull_drag_force` (used by `vehicle.py`).
- `buoyancy.py` — `buoyancy_force` (constant volume; only `sim.py` and tests) and `gravity_force` (both use it).

## `hydrofoil/control/`
- `pid.py` — `PID` (derivative-on-measurement, clamp + conditional anti-windup), `HydrofoilController` (3 PIDs → 3 `Actuator`s, ±20° limit).
- `surge.py` — `SurgeController` (PID, ±`max_thrust`).
- `actuator.py` — `DEFAULT_ACTUATOR_PARAMS`, `Actuator` (delay → rate limit → first-order lag).

## `hydrofoil/waves/airy.py`
`AiryWave(amplitude, wavelength, period, direction, phase)` with `elevation(x,y,t)` and `orbital_velocity(x,y,z,t)`;
`CalmWater` subclass returns zeros.

## `tests/`
- `test_forces.py` — polar loads/symmetry/zero-lift/slope(0.10–0.12/deg)/clamp; rotation orthogonality; J(0)=I;
  skew antisymmetry; M symmetric/PD; zero-force zero-accel; foil zero-speed, lift sign, restoring lift, drag sign; buoyancy/gravity balance.
- `test_actuator.py` — settling, rate limit, delay, reset, controller output is actuator-filtered.
- No tests for: `vehicle.py`, hull, waves, free-surface factor, added mass, PID, surge, UDP, dashboard.

## `data/`
`foil_tables/naca0009_xflr5_re2700000.dat` — XFLR5 v6.61, Re 2.7e6, Mach 0, Ncrit 9. Columns α[deg] CL CD,
about 137 rows, α = −10° … +17.8°, CL_max ≈ 1.42 at 14.4°. Everything else in `data/` is gitignored generated output.
