# Hydrofoil Sim

Python 6-DOF physics simulation (Fossen notation) of a fully-submerged T-foil
hydrofoil, with a native MuJoCo app (3D view, sliders, live plots, actuator loads). Built for UBC Okanagan Marine Robotics Hydrofoil Club.

## What's here

- `hydrofoil/` — the Python simulation:
  - `vehicle.py` (`VehiclePhysics`) is the canonical 6-DOF physics, strip-theory foils,
    distributed hull hydrostatics, wave coupling, and closed-loop stepping.
  - `control/` holds the takeoff PID (`pid.py`), surge PID (`surge.py`), scheduled
    delay-aware MIMO LQR (`lqr.py`), and flight-mode supervisor (`supervisor.py`).
  - `sensors/` and `estimation/` model noisy rate/latency/dropout channels (`models.py`)
    and the `HeaveFilter` + scheduled `KalmanEstimator` (`kalman.py`).
  - `actuation/` models the moteus/BLDC/gearbox surface drives (`servo.py` with
    torque/voltage limits, compliance, backlash, and flap twist; `hinge.py`, `drive.py`, `loads.py`).
  - `viewer.py` and `sim_server.py` are the interactive MuJoCo 3-D app and browser
    launcher / boat-editor dashboard (`http://127.0.0.1:8765`, static assets in
    `hydrofoil/assets/launcher/`). Physics stays in `vehicle.py`.
  - `design.py`, `consistency.py`, `monitor.py`, and `tuning/` provide the boat editor,
    parameter co-dependency checker, flight observer, and phase-aware controller autotuner
    (`--target auto` optimizes PID for takeoff/landing and LQR for foilborne cruise and 3D turns).
- `docs/` — engineering documentation, audits, and design studies (start at [docs/README.md](docs/README.md)).
- `tests/` — pytest suite covering physics, actuation, compliance, linearisation, LQR/estimation, supervisor, and tools.
- `tools/` — verification and sizing scripts (`audit.py`, `actuator_budget.py`, `flap_design.py`, `phase_trials.py`).
- `requirements.txt` — Python deps.

## Running it

Double-click `run_sim.bat` on Windows (or run `./run_sim.sh` on macOS/Linux) for guided `.venv` setup and launch, or run from a terminal:

```bash
pip install -r requirements.txt

# Tests (both work; tests/conftest.py adds the repo root to sys.path)
pytest tests/
python -m pytest

# Parameter co-dependency check
python -m hydrofoil.consistency

# Web launcher, boat editor, and autotuner dashboard (http://127.0.0.1:8765)
python -m hydrofoil.sim_server

# Or launch the 3-D MuJoCo viewer directly. Click the window, then:
#   Up/Down = height, Left/Right = bank, Space = level, [ ] (-/+) = speed, F = foil/land, R = reset, C = Auto/LQR/PID, U = m/s / km/h / mph, D = dashboard
python -m hydrofoil.viewer [--preset model|manned] [--supervisor] [--controller auto|lqr|pid] [--sensors ideal|noisy] [--waves] [--speed 1.0]

# Actuator budget, flap/drive design study, and per-phase mission sweep
python tools/actuator_budget.py
python tools/flap_design.py static
python tools/phase_trials.py
```

## Current state & caveats

- **Default control & estimation:** Both `DEFAULT_PARAMS` (25 kg model) and
  `MANNED_PARAMS` (275 kg craft) default to `controller="lqr"` (PID takeoff with
  bumpless latched handover to speed-scheduled delay-aware LQR with coordinated-turn
  feedforward and roll-prioritized split-front-flap allocation); `--controller auto`
  (default in the web launcher and launch scripts) selects **Auto Combined mode**,
  which runs PID while hull-wetted, blends smoothly into LQR once foilborne, and
  automatically hands back to PID when slowing down or settling onto the water.
  Both presets default to `actuator_model="servo"` and `sensors="ideal"`; set
  `sensors="noisy"` for realistic MEMS/ranging noise, dual height sensor voting
  (`height` + `height_aux`), GNSS vertical-velocity fallback, and `HeaveFilter` +
  `KalmanEstimator`.
- **3D turning & supervisor:** The physical control layout uses two split front flaps
  (`left`, `right`) for heave + roll and a single all-moving rear elevator (`elev`)
  for pitch only (no rudder). `supervisor=True` (or `--supervisor` in the viewer)
  enables mode management (`DISPLACEMENT` → `TAKEOFF` → `FOILING` → `LANDING` /
  `EMERGENCY_LANDING`), dynamic safe bank-angle clamping (`safe_bank_limit`, up to
  `±15°` in calm cruise and automatically narrowed with speed, height, and wave
  amplitude to protect outer wingtip submergence and inner chine clearance),
  smooth `12°/s` bank ramping, and drive/sensor/attitude fault handling
  ([control/supervisor.py](hydrofoil/control/supervisor.py),
  [docs/21-per-phase-controller-trials.md](docs/21-per-phase-controller-trials.md),
  [docs/23-code-audit-and-3d-turning.md](docs/23-code-audit-and-3d-turning.md)).
- **Assumptions & placeholder parameters:** Every assumed, placeholder, derived, and
  tuned parameter across the simulation is catalogued with its provenance and
  hardware replacement plan in [docs/22-assumptions-and-provenance.md](docs/22-assumptions-and-provenance.md)
  (alongside [docs/04-constants-and-parameters.md](docs/04-constants-and-parameters.md) and
  [docs/20-flap-and-drive-design.md](docs/20-flap-and-drive-design.md)); `design.py`
  propagates dimension/mass/torque edits consistently across downstream inertias,
  damping, shaft sizing, and base gains.
- **Known physical simplifications:** Monochromatic Airy waves, empirical shallow-breach
  and strut-sideslip ventilation hysteresis, span-averaged free-surface loss, and
  displacement Froude hump without unsteady cavitation or slamming in `hull.py`.
  See [docs/11-known-issues.md](docs/11-known-issues.md),
  [docs/19-linearisation-lqr-estimation.md](docs/19-linearisation-lqr-estimation.md),
  and [docs/23-code-audit-and-3d-turning.md](docs/23-code-audit-and-3d-turning.md).
