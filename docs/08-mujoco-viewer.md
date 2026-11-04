# 08 — MuJoCo Viewer (replaces the Unity visualizer)

The Unity project, the UDP bridge (`stream.py`, `control_uplink.py`) and `sim_realtime.py` were removed on branch `mujoco-viewer`
(recoverable from git history, e.g. `git show 841d501:unity-visualizer/...`). The 709 MB hull `.blend` and the WSL/Unity networking
setup are no longer needed.

## Design
`hydrofoil/viewer.py` runs `VehiclePhysics` **in-process** and uses MuJoCo only as a renderer: every frame the pose is written into
`qpos` and `mj_forward` is called; `mj_step` is never used (gravity is off in the MJCF). Python stays the sole physics authority.

- `build_mjcf(params)` generates the model from the same params dict as the sim, so geometry in `DEFAULT_PARAMS` shows up automatically:
  hull box (`hull_offset` above CoG), main strut, cross bar, left/right main foil halves each with a hinged 40 % flap, elevator on its own strut
  (all-moving, hinged), red CoG marker, an infinite checker seabed at z = -3 m for motion cues, and a translucent water plane (mocap body) that follows the vehicle in x/y and rides the wave elevation.
- `MujocoPose.apply(state, cmds, wave_elev)` maps NED/body to MuJoCo Z-up. World: `D = diag(1,-1,-1)`; body frame (x fwd, y left, z up) vs sim body (x fwd, y stbd, z down): `B = diag(1,-1,-1)`;
  `R_mj = D R_ned B`, `p_mj = D p_ned`. Flap hinges use `-delta` because a positive MuJoCo rotation about +y (left) raises the trailing edge.
  Verified numerically: foil, elevator and hull positions match `eta + R r_attach` mapped through `D` to 4 decimals; a +15 deg main flap lowers the trailing edge.
- `Pilot` replaces the Unity stick. Keys are handled via the viewer's `key_callback` (GLFW key codes), one step per key event:
  Up/Down height +-0.02 m (-0.3 to 1.0), Left/Right bank +-1 deg (+-8 deg, springs back at 10 deg/s), `[` `]` speed -/+0.25 m/s (0 to 10), `R` reset.
- Loop: `target_t = wall_elapsed * speed`; step physics until caught up (cap 50 steps/frame, backlog dropped), apply pose, update the on-screen text, `sync`, ~60 fps.
  A tracking camera follows the vehicle (distance 4.5 m, az 140, el -18); the usual MuJoCo mouse controls still work.
- Overlay text shows time, FOILBORNE/DISPLACEMENT, speed vs target, hull height vs target, roll/pitch, the three flap angles, thrust/draft and `phi_ref`.

## Run
```bash
pip install mujoco          # already in requirements.txt
python -m hydrofoil.viewer [--waves] [--speed 1.0] [--duration 600]
```
Click the viewer window so it has keyboard focus. Needs a desktop with OpenGL (runs natively on Windows, no WSL).

## Tested
Model compiles (nq = 10), pose and flap-sign checks above, and a 6 s smoke run of the viewer exited cleanly. **Not tested:** key events on a real keyboard
(key codes are GLFW's: 265/264 up/down, 262/263 right/left, 91/93 brackets, `R`), long runs, `--waves` visual quality, other platforms.

## Known limits / ideas
- Visual only: no wave-surface mesh (a flat plane moves vertically with the wave elevation at the vehicle), no spray, no flow lines.
- The viewer shares `DEFAULT_PARAMS` but is not wired to the dashboard sliders; run one or the other. A shared live-params object could unify them.
- The dashboard (`hydrofoil.dashboard`) remains the tool for plots. The same pose code could render offline video with `mujoco.Renderer`.
- The roll loop issue in doc 13 will be very visible in this viewer: banking more than a degree or two with default gains will make the boat roll over and drop back into the water.
