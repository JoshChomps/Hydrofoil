# 11 — Known Issues, Inconsistencies, Stale Docs

Found by reading the code on 2026-11-04. Severity: **H** affects results/safety, **M** confusing/wrong in an edge case, **L** cosmetic.

## Stale or contradictory documentation
| # | Sev | Issue |
|---|---|---|
| D1 | M | `hydrofoil/PROJECT_STATE.md` (2026-09-17) says "No added mass", "No test suite", and describes foils without flap effectiveness / finite-span / free-surface / induced drag. The code now has all of those and two test files. README is the current source. |
| D2 | M | `PROJECT_STATE.md` says the dashboard JS clamps bank at ±8° (`ROLL_MAX`); the code has `ROLL_MAX = 10.0` and the Bank sliders go ±10°. Only the UDP uplink and Unity enforce ±8°. |
| D3 | L | README mentions `notebooks/` (empty, not in the repo). |
| D4 | L | `test_forces.py` comment says "119 rows"; the polar has ~137. Test only asserts > 100. |
| D5 | L | `stream.py`/`PythonSimDriver.cs` comments mention `SimState.cs` / `HydrofoilScene.cs`, which do not exist (the class is `SimPacket`). |
| D6 | L | `PROJECT_STATE.md` says `sim.py` saves to `../data/` (relative to `hydrofoil/`); that resolves to the repo-root `data/`, which is gitignored apart from `foil_tables/`. |

## Code issues
| # | Sev | Issue |
|---|---|---|
| C1 | M | `requirements.txt` lacks `plotly` (used by `viz.py`, `sim.py`) and `pytest`. |
| C2 | M | **Surge PID derivative sign**: derivative-on-measurement assumes error and measurement move together. For surge (`e = ref − u`) they move oppositely, so `Kd > 0` adds positive feedback. Latent: default Kd = 0 (UI allows up to 20). |
| C3 | L | `sim.py`/`viz.py` write into `data/` without creating it. It exists on a fresh clone only because `data/foil_tables/` is tracked; deleting `data/` would break them. |
| C4 | M | Polar table covers −10°…+17.8°. Positive and negative sides are not symmetric beyond ±10°, so a large negative AoA is clamped earlier than a positive one. Elevator (effectiveness 1.0, ±20°) can reach beyond the table on the positive side as well. |
| C5 | M | AoA ignores body pitch and mounting incidence (`aoa_offset` always 0). May be intended, but means foil incidence depends only on velocity direction + flap. |
| C6 | M | `step()` returns `draft` from the pre-step η and the dashboard publishes pre-step `eta`/`nu` — one-step (10 ms) lag in displayed data. Negligible, but the "foilborne" flag lags by one step. |
| C7 | M | Wave orbital velocity is evaluated once at the body origin depth and held for foils up to 0.5 m deeper/1.2 m aft; wave λ and T are independent (non-dispersive combos possible). |
| C8 | L | Hull buoyancy and weight both act at the body origin; no restoring moment from the hull (pitch/roll stiffness while in displacement mode comes only from the foils). |
| C9 | L | `buoyancy_force` imports `R_body_to_ned` inside the function; minor performance cost per RK4 stage. `np.linalg.solve` of a constant M each call could be a precomputed inverse. |
| C10 | L | Actuator delay is specified in steps, so the delay in seconds changes with `dt`. |
| C11 | L | Importing `DEFAULT_PARAMS` from `sim_realtime.py` pulls in FastAPI/uvicorn via `dashboard.py`. |
| C12 | L | `SimRunner.set_live_params` locks but `_run` reads `p[...]` unlocked (benign race on floats). |
| C13 | L | Dashboard server binds `0.0.0.0` with no authentication; POST endpoints can change/stop the sim from the network. |
| C14 | L | `sim_realtime.py` converts θ with `3.14159265` literal instead of `np.degrees`. |
| C15 | L | `PilotUplink.stop()` joins with a timeout; socket close from another thread relies on `OSError` to exit. |
| C16 | L | `get_windows_host_ip` uses `shell=True` `ip route`; on native Windows this silently falls back to 127.0.0.1 (works, but the `[SimStreamer] Sending to` line is the only indication). |

## Modelling concerns worth keeping in mind
- Heave added mass (≈24 kg) is about equal to the vehicle mass, so the heave dynamics are roughly twice as heavy as the rigid body alone; combined with 0.21 flap effectiveness this reduces heave authority — likely the reason behind the README's "under-authoritative" warning.
- `heave_damping = 50 N·s/m` is the dominant non-foil damping and is a tuning knob, not physics.
- Lateral offset 0.35 m with a 0.6 m half-span means the two "halves" overlap geometrically if they are meant to be adjacent panels from the centreline (centres at ±0.3 m). It is a lever-arm choice, not a layout check.

## Unity issues
| # | Sev | Issue |
|---|---|---|
| U1 | H (for new machines) | Auto-launch uses `/home/jleathem/code/unity` and distro `Ubuntu`. |
| U2 | M | `Floater.cs` logs every FixedUpdate when underwater (only when enabled). |
| U3 | M | `SimulinkTCPReceiver` uses `Thread.Abort` (unsupported on modern runtimes) and a byte-order TODO. |
| U4 | L | Both URP and HDRP packages/assets present. |
| U5 | L | Several dead scripts (`FoilData`, `HightBalancer`, `HydrofoilElement`, `HydrofoilController`). |

## What was not verified
- **Nothing was executed.** My attempt to run `pytest` and a headless takeoff probe was blocked by the environment's permission check, so test status, takeoff behaviour, stability limits, and all numbers in docs 03 §7 and 05 §3 are from reading the code and hand calculation.
- The stability claims (Ki divergence, bank 12°, fuzz divergence) are quoted from upstream `PROJECT_STATE.md`/code comments, not reproduced.
- Unity project was not opened; scene wiring (which objects hold which script references) was not inspected.
- The raw `.unity` scene YAML and HDRP/URP assets were not read.
