# 09 — User Manual

## 1. Install

Requirements: Python ≥ 3.10 (tested 3.12).

```bash
cd "D:/Coding_Projects/Hydrofoil Sim"
python -m venv .venv
.venv/Scripts/activate            # Windows (bash: source .venv/Scripts/activate)
pip install -r requirements.txt   # numpy scipy matplotlib plotly mujoco pytest
```
Run all commands from the repo root so `import hydrofoil...` resolves (there is no installable package / `pyproject.toml`).

## 2. Run the app (MuJoCo)

```bash
python -m hydrofoil.viewer                         # calm water, real time
python -m hydrofoil.viewer --waves --speed 2       # Airy wave (A 0.05, λ 5, T from dispersion), 2x speed
python -m hydrofoil.viewer --waves --wave-dir 90   # beam seas (excites roll)
```
Sequence: the vehicle starts at rest floating (draft ≈ 4 cm, pitch 2°). The surge PID accelerates toward the target speed and
the heave PID raises the flaps; the mode text flips from **DISPLACEMENT** to **FOILBORNE** once the hull is dry.

- **Keys** (click the window first): Up/Down height, Left/Right bank (springs back), `Space` level wings, `[` `]` (`-` `+`) speed, `F` foil/land, `R` reset, `C` cycle `Auto Combined -> LQR -> PID`, `U` cycle `m/s -> km/h -> mph`, `D` open web dashboard (`http://127.0.0.1:8765`).
- **Launchers**: double-click `run_sim.bat` on Windows or run `./run_sim.sh` on macOS/Linux (automatically sets up `.venv` with all dependencies including Pillow and selects `mjpython` on macOS for MuJoCo).
- **Sliders**: right panel → **Control** (and mirrored in the web dashboard Helm): references, waves, PID gains and the drive's gear ratio / bus voltage, all live.
- **Plots** on the left: height vs target, roll/pitch, surface angles, drive currents (last 15 s).
- **Top-right text**: state, and per drive the hinge moment, motor current and bus power, with a limit flag.
Geometry, empty hull mass, and interactive payload/CG placement: edit in the web dashboard's **Boat editor** tab (drag mass circles or edit coordinates, saves to `local/overrides.json` and relaunches cleanly) or via `hydrofoil/design.py`. Details in doc 08.

## 3. Actuator budget

```bash
python tools/actuator_budget.py                  # 5 scenarios at N = 20, then a gear-ratio sweep
python tools/actuator_budget.py --ratios 15 20 25 --vbus 36
```
Prints per drive the deflection range, hinge moment, output torque, rate, current, voltage, bus power, regen energy and
pass/fail against the moteus/motor/gearbox/shaft limits. Assumptions and interpretation: doc 17.

## 4. Headless batch (legacy model)

```bash
python -m hydrofoil.sim
```
Runs closed-loop and zero-flap open-loop 90 s cases from rest on the **same physics as the app** (it is now a thin wrapper over `VehiclePhysics`), writes PNGs and HTML to `data/` and opens them (WSL-aware; falls back to the default browser).

## 5. Tests

```bash
pytest tests/ -v
```
42 tests; the closed-loop ones (takeoff, roll rejection, bank tracking, oblique waves) take about a minute in total.
Cover polar loading/symmetry, kinematics, rigid body, foil force signs, buoyancy/gravity, actuator and controller filtering. They do not cover `vehicle.py`, hull, PID, or waves.

## 6. Reading the telemetry

| Field | Meaning |
|---|---|
| Depth | CoG NED z (positive down). Target = `hull_offset − flying_height` |
| Draft | Hull-bottom depth below local water surface; 0 = foilborne |
| d_main / d_elev / d_roll | actuator-filtered flap angle [deg]; left foil = d_main − d_roll, right = d_main + d_roll |
| Thrust | surge PID output [N], capped ±`max_thrust` |
| Heading | ψ in degrees; drifts as a result of banked turns |

## 7. Tuning workflow (current practice)
- **Automated Phase-Aware Tuning (Recommended):** Open the **Autotuner** tab in `sim_server` (or run `python -m hydrofoil.tuning.autotune --preset manned --target auto`) to tune PID gains on Rising/Falling phases (`takeoff`, `supervised`, `landing`) and LQR weights on Cruising & 3D Turning (`head`, `following`, `shift`, `noisy`, `turn`), saving both to `local/overrides.json`.
1. Calm water, default geometry. Change one gain at a time on the Live tab → Apply & Restart.
2. Check takeoff time, depth overshoot, pitch settling, roll settling.
3. Add waves; confirm no divergence for ≥ 90 s.
4. Keep roll Ki = 0. Re-run the bank sweep (0–12°) in calm water if the roll loop or limits are touched.
5. After any change to `foil.py` constants the existing gains are suspect — retune (README warns this has not been done).

## 8. Common problems

| Symptom | Cause / fix |
|---|---|
| `ModuleNotFoundError: hydrofoil` | Run from repo root with `python -m` |
| `ModuleNotFoundError: plotly` | `pip install plotly` (only `sim.py`/`viz.py`) |
| Dashboard blank charts | No internet for the Plotly CDN |
| Viewer keys do nothing | Click the viewer window first; MuJoCo's own hotkeys share the keyboard |
| Vehicle never lifts | Target speed too low (< ~3 m/s on current constants) or `max_thrust` too small for hull drag |
| Violent attitude then "reset" look | Safety net rolled back after non-finite state; reduce combined inputs |
| Changing a slider does nothing | It needs **Apply & Restart** (only height and roll are live) |
