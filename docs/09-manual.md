# 09 — User Manual

## 1. Install

Requirements: Python ≥ 3.10; Unity 6000.3.6f1 only if you want the 3D view.

```bash
cd "D:/Coding_Projects/Hydrofoil Sim"
python -m venv .venv
.venv/Scripts/activate            # Windows (bash: source .venv/Scripts/activate)
pip install -r requirements.txt   # numpy scipy matplotlib fastapi uvicorn[standard]
pip install plotly pytest         # not in requirements.txt but needed for sim.py/viz.py and tests
```
Run all commands from the repo root so `import hydrofoil...` resolves (there is no installable package / `pyproject.toml`).

## 2. Run the dashboard (main tool)

```bash
python -m hydrofoil.dashboard
```
Open <http://localhost:8000> (needs internet for the Plotly CDN script). Press **Run**.

Sequence: the vehicle starts at rest floating (draft ≈ 4 cm, pitch 2°), surge PID accelerates toward the target speed, heave PID
raises the flaps to generate lift, the hull rises until draft = 0 → status flips from **DISPLACEMENT** to **FOILBORNE**.

### Panels
- **Left, LIVE CONTROLS tab**: ① flight target (hull height, target pitch, target roll), ② propulsion (target speed, speed gains, max thrust),
  ③–⑤ PID gains for heave/pitch/roll, ⑥ waves.
- **VEHICLE CONFIG tab**: mass/inertia, hull, main foil, elevator, dt and duration.
- **Right**: roll front-view, side profile, depth gauge; charts (depth, speed & thrust, angles, flap deflections, last 20 s); telemetry strip.

### What applies live vs. needs restart
| Change | Live? |
|---|---|
| Hull height (`flying_height`), target roll (`phi_ref`), keyboard stick | **live** |
| Everything else (speed target, gains, waves, geometry, `theta_ref`, dt) | press **Apply & Restart** |
Run/Stop/Reset: Reset restores `DEFAULT_PARAMS` and clears charts. The run auto-stops at `t_end` (default 60 s).

### Keyboard
W/↑ climb, S/↓ dive (0.15 m/s of target), A/← bank left, D/→ bank right (20 °/s, ±10°, springs to 0 on release). Stay within ±8° to keep inside the verified stable region.

## 3. Stream to Unity

```bash
python -m hydrofoil.sim_realtime --host 127.0.0.1          # native Windows, Unity on the same machine
python -m hydrofoil.sim_realtime --waves --speed 1         # with Airy waves (A 0.05, λ 5, T 2)
```
Then press Play in Unity. Details in doc 08. Console prints a line per sim-second:
`t, z, draft [MODE], u (target), theta, thrust, delta_main`.

## 4. Headless batch (legacy model)

```bash
python -m hydrofoil.sim
```
Runs closed-loop and open-loop 20 s cases, writes PNGs and HTML to `data/` and opens them (WSL-aware; falls back to the default browser). This is the old pinned-speed model; not representative of `vehicle.py`.

## 5. Tests

```bash
pytest tests/ -v
```
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
| Unity shows no movement | Wrong `--host`, firewall, Python not running, or Unity window not receiving (set `runInBackground` already on); check `_packetsReceived` in the inspector |
| Unity auto-launch fails | Fix `wslDistro` / `simCommand` path, or disable `autoLaunchSim` |
| Hull invisible | Missing 709 MB `.blend` |
| Vehicle never lifts | Target speed too low (< ~3 m/s on current constants) or `max_thrust` too small for hull drag |
| Violent attitude then "reset" look | Safety net rolled back after non-finite state; reduce combined inputs |
| Changing a slider does nothing | It needs **Apply & Restart** (only height and roll are live) |
