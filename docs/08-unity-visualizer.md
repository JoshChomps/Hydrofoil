# 08 — Unity Visualizer

Project: `unity-visualizer/` — Unity **6000.3.6f1**, packages include Input System 1.18, HDRP 17.3, URP 17.3, Shader Graph, Timeline
(manifest has both render pipelines; scene settings exist for both). Input handling is set to **Both**.
The repo contains a trimmed project (default HDRP/terrain sample content removed). Six dated screenshots are in `Assets/Screenshots/`.

## Required missing file
`Assets/Foil/Mockup4_JacobRig_Merged.blend` (709 MB) is excluded by `.gitignore`. Per README it lives on the club
"hydro foil drive" under mech → CAD files. Without it the hull does not render; scripts, scene and physics still work.

## Live path (what actually runs)

### `HydrofoilSim/PythonSimDriver.cs` (398 lines)
- Public refs: `shipTransform`, `leftFoil`, `rightFoil`, `elevatorFoil`; `port 5005`; `pythonHost 127.0.0.1`, `pilotPort 5006`.
- `Start()`: sets `Application.runInBackground = true` (without it the editor stalls when unfocused), switches `SimModeController` to PythonDriven, opens a UDP listener thread, optionally launches Python.
- **Auto-launch** (`autoLaunchSim = true`): runs `wsl.exe -d <wslDistro> -e bash -lc "<simCommand>"`. Defaults: distro `Ubuntu`, command
  `cd /home/jleathem/code/unity && python3 -u -m hydrofoil.sim_realtime --host 127.0.0.1 --duration 86400`.
  **These are machine-specific and must be edited (or auto-launch disabled) on any other machine.** `OnDestroy` kills the `wsl.exe` process and runs `pkill -f hydrofoil.sim_realtime` in WSL.
- `Update()`: applies the newest packet, polls pilot input.
- Pilot keys (legacy `Input`): ↑/↓ height (0.15 m/s, holds), W/S target speed (1 m/s², holds), A/D bank (6 °/s, ±8°, springs back at 10 °/s).
- Send rate 20 Hz.
- **Coordinate mapping** NED → Unity: `position = (y, −z + waterSurfaceY, x)`; `rotation = Euler(−θ°, ψ° + modelYawOffsetDeg, φ°)`; `modelYawOffsetDeg = −90` fixes the mesh's authored forward axis.
- Foil flaps: left `localRotation = Euler(−(d_main − d_roll), 0, 0)`, right `Euler(−(d_main + d_roll), 0, 0)`, elevator `Euler(−d_elev,0,0)`.
- IMGUI HUD top-right: Front Left / Front Right / Back Wing flap angles. Inspector shows live state (time, depth, pitch, roll, speed, deltas, thrust, draft, foilborne, wave_elev, packets, pilot targets).
- Not used in the scene logic: `v`, `w` (stored in the packet only), `wave_elev` (shown, not applied to the water mesh).

### `HydrofoilSim/SimModeController.cs`
Enum `Arcade | PythonDriven`. In PythonDriven: Rigidbody `isKinematic = true`, `Floater`s and `TwinMotorController` disabled. Prevents the legacy physics fighting the transform writes.

## Legacy / arcade scripts (not part of the Python-driven path)

| Script | Purpose | Status |
|---|---|---|
| `SimulinkTCPReceiver.cs` | TCP listener (port 55000) for Simulink doubles → transform/flaps | Superseded by `PythonSimDriver` ("drop-in replacement"); uses `Thread.Abort` |
| `Floater.cs` | Arcade buoyancy at floater points (+ `Debug.Log` every FixedUpdate when underwater — noisy) | Disabled in Python mode |
| `HydrofoilController.cs` | Arcade WASD controller with float/foil points and v² lift (New Input System) | Legacy |
| `HydrofoilElement.cs` | Per-foil lift/drag from an `AnimationCurve` | Legacy |
| `TwinMotorController.cs` | Twin-motor thrust (`maxThrust` 5000 N in code vs 50 N cited by Python) | Disabled in Python mode |
| `FoilData.cs`, `HightBalancer.cs` | Empty stubs ("HightBalancer" typo for height; has an unused `ultraSoundSencer` field) | Unused |

Other assets: `Scenes/SampleScene.unity`, `Materials/Water test.mat`, `Materials/New Actions.inputactions`, `InputSystem_Actions.inputactions`, HDRP/URP settings, three `TerrainData_*.asset`, `New Terrain.asset`, `Readme.asset`.

## How to set up
1. Obtain the `.blend` and drop it at `Assets/Foil/` (Blender not required if Unity can import it; the file is Blender-native).
2. Open `unity-visualizer/` in Unity Hub with editor 6000.3.6f1.
3. Select the object with `PythonSimDriver`; confirm `shipTransform`, `leftFoil`, `rightFoil`, `elevatorFoil` are wired.
4. Either leave `autoLaunchSim` on after editing `wslDistro` and `simCommand`, or turn it off and start Python yourself:
   `python -m hydrofoil.sim_realtime --host 127.0.0.1` (native Windows) or from WSL with the default host detection.
5. Press Play. Use ↑/↓, W/S, A/D.

## Networking gotchas (documented upstream)
- Under WSL2 **mirrored networking**, the auto-detected gateway IP does not route; use `--host 127.0.0.1`.
- Windows firewall may prompt for UDP 5005/5006.
- Python → Unity packet loss is silent; Unity keeps the last packet.
