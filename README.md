# Hydrofoil Sim

Python 6-DOF physics simulation (Fossen notation) of a fully-submerged T-foil
hydrofoil, with a native MuJoCo app (3D view, sliders, live plots, actuator loads). Built for UBC Okanagan Marine Robotics Hydrofoil Club.

## What's here

- `hydrofoil/` — the Python simulation. `vehicle.py` is the canonical physics
  (6-DOF rigid body, strip-theory foils, hull buoyancy/drag, wave coupling,
  PID control). `viewer.py` is the interactive MuJoCo app (render + UI only;
  the physics stays in `vehicle.py`). `actuation/` models the moteus/BLDC/gearbox
  surface drives (hinge moment, current, voltage, stress; docs/17).
  `params.py` holds `DEFAULT_PARAMS`.
- `docs/` — full documentation, math audit and roadmap (start at `docs/README.md`).
- `tests/` — pytest unit tests (`test_forces.py`, `test_actuator.py`).
- `notebooks/` — currently empty, reserved for analysis notebooks.
- `requirements.txt` — Python deps.

## Running it

```bash
pip install -r requirements.txt

# Tests
pytest tests/

# The app (MuJoCo). Click the window, then:
#   Up/Down = height, Left/Right = bank, [ ] = speed, R = reset; sliders in the right panel -> Control
python -m hydrofoil.viewer [--waves] [--speed 1.0]

# Actuator budget (hinge moment -> motor current/voltage, gear-ratio sweep)
python tools/actuator_budget.py
```

## Known state / heads-up for anyone picking this up

- Control is three independent PID loops (heave/pitch/roll), not yet a MIMO
  controller — see `hydrofoil/control/pid.py`. Roll `Ki` must stay `0`
  (verified to diverge otherwise under wave disturbance). Pilot bank input
  is clamped to ±8° (`hydrofoil/control_uplink.py`) — commanded bank beyond
  that has been verified to diverge.
- The foil force model (`hydrofoil/forces/foil.py`) includes flap
  effectiveness, finite-span, free-surface, and induced-drag corrections.
  PID gains have **not** been retuned against these corrections yet — expect
  the response to feel under-authoritative until that happens.
- No state estimation or sensor noise modeling exists yet (`estimation/`,
  `sensors/` are empty packages) — the sim feeds every controller exact
  ground-truth state.
- No linearized state-space (A/B) model of the plant exists yet — that's the
  prerequisite for the planned LQR/MPC work and hasn't been started.
