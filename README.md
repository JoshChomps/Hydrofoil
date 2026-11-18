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
  - `viewer.py` is the interactive MuJoCo 3-D app (render + sliders + live plots +
    drive telemetry; physics stays in `vehicle.py`).
  - `design.py`, `consistency.py`, `monitor.py`, and `tuning/` provide the boat editor,
    parameter co-dependency checker, flight observer, and controller autotuner.
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

# The app (MuJoCo). Click the window, then:
#   Up/Down = height, Left/Right = bank, [ ] = speed, F = foil/land (--supervisor), R = reset, C = PID/LQR
python -m hydrofoil.viewer [--preset model|manned] [--supervisor] [--controller lqr|pid] [--sensors ideal|noisy] [--waves] [--speed 1.0]

# Actuator budget, flap/drive design study, and per-phase mission sweep
python tools/actuator_budget.py
python tools/flap_design.py static
python tools/phase_trials.py
```

## Current state & caveats

- **Default control & estimation:** Both `DEFAULT_PARAMS` (25 kg model) and
  `MANNED_PARAMS` (275 kg craft) default to `controller="lqr"` (PID takeoff with
  bumpless latched handover to speed-scheduled delay-aware LQR) and
  `actuator_model="servo"`. Sensors default to `"ideal"`; set `sensors="noisy"`
  for realistic MEMS/ranging noise, dual height sensor voting (`height` +
  `height_aux`), GNSS vertical-velocity fallback, and `HeaveFilter` + `KalmanEstimator`.
- **Supervisor:** `supervisor=True` (or `--supervisor` in the viewer) enables mode
  management (`DISPLACEMENT` → `TAKEOFF` → `FOILING` → `LANDING` / `EMERGENCY_LANDING`),
  speed/height/banked-clearance envelope clamps, and drive/sensor/attitude fault handling
  ([control/supervisor.py](hydrofoil/control/supervisor.py),
  [docs/21-per-phase-controller-trials.md](docs/21-per-phase-controller-trials.md)).
- **Placeholder parameters:** Mass, inertias, hull drag/damping, strut chords, and
  several drive/compliance numbers remain placeholders pending hardware measurements
  (see [docs/04-constants-and-parameters.md](docs/04-constants-and-parameters.md) and
  [docs/20-flap-and-drive-design.md](docs/20-flap-and-drive-design.md)); `design.py`
  propagates dimension/mass/torque edits consistently across downstream inertias,
  damping, shaft sizing, and base gains.
- **Known physical simplifications:** Monochromatic Airy waves, empirical shallow-breach
  ventilation hysteresis and displacement Froude hump without unsteady cavitation or
  slamming in `hull.py`. See [docs/11-known-issues.md](docs/11-known-issues.md) and
  [docs/19-linearisation-lqr-estimation.md](docs/19-linearisation-lqr-estimation.md).
