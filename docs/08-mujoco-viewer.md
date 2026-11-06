# 08 — MuJoCo App (the one interactive front-end)

`python -m hydrofoil.viewer` is now the only interactive tool. It replaced the Unity visualizer (removed 2026-11-03) and the
browser dashboard (removed 2026-11-05; recover with `git show 58c148e:hydrofoil/dashboard.py`). The motivation: the two front-ends
ran the same physics but in separate processes with separate live params and duplicated pilot logic, so they could drift apart.
There is now one process, one params dict and one pilot.

## Design
`hydrofoil/viewer.py` runs `VehiclePhysics` **in-process** and uses MuJoCo only as renderer and UI. Every frame the pose is
written into `qpos` and `mj_forward` is called. `mj_step` is never used (gravity is off in the MJCF). Python stays the sole
physics authority.

- **Geometry**: `build_mjcf(params)` generates the model from the same params dict as the sim: hull box (`hull_length` × `hull_beam`,
  `hull_x`, `hull_offset` above CoG), main strut, cross bar, left/right main-foil halves each with a hinged `main_flap_frac` flap,
  hinged all-moving elevator, red CoG marker, checker seabed at z = −3 m, and a translucent water plane (mocap) riding the wave elevation.
- **Sliders**: the `SLIDERS` table becomes zero-gain MuJoCo actuators. They exist only so the viewer's right-hand **Control** panel
  draws sliders; nothing is ever stepped. Their `ctrl` values are copied into the live params every frame. Covered: height/bank/speed
  references, wave amplitude/length/direction (amplitude 0 = calm; a change rebuilds the wave object), live PID gains (heave
  Kp/Ki/Kd, pitch Kp/Kd, roll Kp/Kd; written straight into the running PIDs, so no restart is needed), and the drive's gear ratio and bus voltage.
  Changing Ki mid-run rescales the stored integral (the PID stores ∫e, not Ki∫e), so expect a small bump.
- **Keys** (`Pilot`) write into the same slider values, so keys and sliders are one source of truth: Up/Down height ±0.02 m,
  Left/Right bank ±1° (±8°, springs back at 10°/s; a bank set with the slider holds), `[` `]` speed ∓0.25 m/s, `R` reset.
- **Plots** (`Plots`, `MjvFigure` through `viewer.set_figures`): left edge, rolling 15 s at 30 Hz. Height vs target, roll/pitch,
  the three physical surface angles, and the three drive currents.
- **Drive read-out** (`DriveMonitor`, doc 17): per step, the hinge moment, motor current and bus power for left flap, right flap and
  elevator, with a limit flag.
- **Pose mapping** (`MujocoPose.apply`): world `D = diag(1,−1,−1)`, body `B = diag(1,−1,−1)`, `R_mj = D R_ned B`, `p_mj = D p_ned`;
  surface hinges get `−delta` of the *physical* (mixed) angles, because positive MuJoCo rotation about +y (left) raises the trailing edge.
- **Loop**: `target_t = wall_elapsed × speed`; step physics until caught up (cap 50 steps/frame, backlog dropped), apply the pose
  under the viewer lock, update text/figures, `sync`, ~60 fps. Tracking camera (4.5 m, az 140, el −18); normal MuJoCo mouse controls.

## Run
```bash
python -m hydrofoil.viewer [--waves] [--wave-dir 90] [--speed 1.0] [--duration 600]
```
Click the window for keyboard focus; open the right panel (Tab or the arrow) → **Control** for sliders. Needs a desktop with OpenGL
(runs natively on Windows). It is not a web server, so it has no `.claude/launch.json` entry.

## Tested
Model compiles with 15 slider actuators; the slider/key/spring-back logic, the drive monitor and the figure builder were exercised
headlessly over 15 s of flight (foilborne, ≈3 A per flap at cruise). The window opened and ran 20 s and exited cleanly.
**Not tested by hand:** dragging the Control sliders (relies on the passive viewer writing UI changes into the shared `MjData.ctrl`),
how the figures look on screen, real key events, other platforms.

## Gone with the dashboard
Vehicle-config editing at runtime (mass, geometry, dt): edit `DEFAULT_PARAMS` and restart. A geometry change needs a rebuilt MJCF
anyway. The HTTP/WebSocket API (doc 07 §3) no longer exists. `viz.py` (offline Plotly report for `sim.py`) is unchanged.

## Known limits / ideas
- Flat moving water plane instead of a wave mesh; no spray, flow lines or track trail.
- Struts are drawn at fixed size; the physics struts use `main_strut_chord`/`elev_strut_chord` and the wetted length.
- Offline video: the same `MujocoPose` with `mujoco.Renderer`.
