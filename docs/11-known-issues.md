# 11 — Known Issues, Inconsistencies, and Resolved Items

> **Current Audit Status (2026-11):** Major baseline items (`C2`, `C4`, `C5`, `C6`, `C7`, `C8`, `C10`, `C11`, `C17`–`C24`) and the subsequent dual-flap Red-Team findings (noisy-mode allocator separation, collinear T-foil yaw push, zero-lift reverse-drive gating, hardcoded `eta_turn`, LQR anti-windup `(ch, j)` rollback, active sideslip CBF unwinding, 3D `cg_z`/`prop_pos_z`, active $C^1$ cavitation, `legacy` `sigma_min`, UI `stops`/`yaw_rate_cmd`/`sup.log`/`drives.cap`/`solve_ms`, and `_CONTROL_KEYS` cache) are now resolved. Open placeholder CFD/hardware parameters are cataloged with swap instructions in [22-assumptions-and-provenance.md §13](22-assumptions-and-provenance.md).

Found by reading the code on 2026-11-04 (original register preserved below). Severity: **H** affects results/safety, **M** confusing/wrong in an edge case, **L** cosmetic.

## Stale or contradictory documentation
| # | Sev | Issue |
|---|---|---|
| D1 | M | `hydrofoil/PROJECT_STATE.md` (2026-09-17) says "No added mass", "No test suite", and describes foils without flap effectiveness / finite-span / free-surface / induced drag. The code now has all of those and two test files. README is the current source. |
| D2 | M | `PROJECT_STATE.md` says the dashboard JS clamps bank at ±8° (`ROLL_MAX`); the code has `ROLL_MAX = 10.0` and the Bank sliders go ±10°. Only the UDP uplink and Unity enforce ±8°. — **FIXED 2026-11-04 (doc 14)** |
| D3 | L | README mentions `notebooks/` (empty, not in the repo). |
| D4 | L | `test_forces.py` comment says "119 rows"; the polar has ~137. Test only asserts > 100. |
| D5 | L | `stream.py`/`PythonSimDriver.cs` comments mention `SimState.cs` / `HydrofoilScene.cs`, which do not exist (the class is `SimPacket`). |
| D6 | L | `PROJECT_STATE.md` says `sim.py` saves to `../data/` (relative to `hydrofoil/`); that resolves to the repo-root `data/`, which is gitignored apart from `foil_tables/`. |

## Code issues
| # | Sev | Issue |
|---|---|---|
| C1 | M | `requirements.txt` lacks `plotly` (used by `viz.py`, `sim.py`) and `pytest`. — **FIXED 2026-11-04 (doc 14)** |
| C2 | M | **Surge PID derivative sign**: derivative-on-measurement assumes error and measurement move together. For surge (`e = ref − u`) they move oppositely, so `Kd > 0` adds positive feedback. Latent: default Kd = 0 (UI allows up to 20). — **FIXED 2026-11-04 (doc 14)** |
| C3 | L | `sim.py`/`viz.py` write into `data/` without creating it. It exists on a fresh clone only because `data/foil_tables/` is tracked; deleting `data/` would break them. |
| C4 | ~~M~~ | ~~Polar table covers −10°…+17.8° (asymmetric past ±10°)~~ — **FIXED** (mirrored symmetric NACA 0009 negative side to −17.8° and added smooth post-stall/reverse-flow Viterna blend in `Foil.generalized_force`). |
| C5 | M | AoA ignores body pitch and mounting incidence (`aoa_offset` always 0). May be intended, but means foil incidence depends only on velocity direction + flap. |
| C6 | M | `step()` returns `draft` from the pre-step η and the dashboard publishes pre-step `eta`/`nu` — one-step (10 ms) lag in displayed data. Negligible, but the "foilborne" flag lags by one step. — **FIXED 2026-11-04 (doc 14)** |
| C7 | M | Wave orbital velocity is evaluated once at the body origin depth and held for foils up to 0.5 m deeper/1.2 m aft; wave λ and T are independent (non-dispersive combos possible). — **FIXED 2026-11-04 (doc 14)** |
| C8b | ~~L~~ | ~~Hull buoyancy had no waterplane-inertia term (pitch stiffness 14× low, roll 23 % low)~~ — **FIXED**, E20/doc 15. |
| C8 | L | Hull buoyancy and weight both act at the body origin; no restoring moment from the hull (pitch/roll stiffness while in displacement mode comes only from the foils). — **FIXED 2026-11-04 (doc 14)** |
| C9 | L | `buoyancy_force` imports `R_body_to_ned` inside the function; minor performance cost per RK4 stage. `np.linalg.solve` of a constant M each call could be a precomputed inverse. |
| C10 | L | Actuator delay is specified in steps, so the delay in seconds changes with `dt`. — **FIXED 2026-11-04 (doc 14)** |
| C11 | L | ~~Importing `DEFAULT_PARAMS` pulled in FastAPI~~ — fixed: now `hydrofoil/params.py`. — **FIXED 2026-11-04 (doc 14)** |
| C12 | L | `SimRunner.set_live_params` locks but `_run` reads `p[...]` unlocked (benign race on floats). |
| C13 | L | Dashboard server binds `0.0.0.0` with no authentication; POST endpoints can change/stop the sim from the network. |
| C14 | L | ~~`sim_realtime.py` literal for π~~ — file removed. |
| C15 | L | `PilotUplink.stop()` joins with a timeout; socket close from another thread relies on `OSError` to exit. |
| C17 | ~~H~~ | ~~PID unit mismatch (rad in, deg out)~~ — **FIXED**, doc 14 E1; gains retuned (E2). |
| C18 | ~~H~~ | ~~Waves symmetric, could not excite roll/yaw~~ — **FIXED** (direction param, per-foil sampling), E13/E14. |
| C19 | ~~M~~ | ~~Munk moment missing~~ — **FIXED**, E7. |
| C20 | M | Heave settling 62 s → ~17 s after retune; still integrator-driven (no feed-forward), O6. |
| C21 | M | The "cascade PID" label in code/docs is a misnomer: three parallel SISO loops, no inner/outer structure. |
| C22 | ~~M~~ | ~~Anti-windup froze integrator at the wrong time~~ — **FIXED**, E3. |
| C23 | M | No trim feed-forward or speed scheduling (lift ∝ V²) — open, O6. |
| C24 | M | Strut side force, hull restoring moment, per-foil wave sampling and λ–T dispersion are now **FIXED** (E10–E15). Still open: wing–tail downwash, flap section moment (O1/O2). |
| C16 | L | `get_windows_host_ip` uses `shell=True` `ip route`; on native Windows this silently falls back to 127.0.0.1 (works, but the `[SimStreamer] Sending to` line is the only indication). |

## Modelling concerns worth keeping in mind
- Heave added mass (≈24 kg) is about equal to the vehicle mass, so the heave dynamics are roughly twice as heavy as the rigid body alone; combined with 0.21 flap effectiveness this reduces heave authority — likely the reason behind the README's "under-authoritative" warning.
- `heave_damping = 50 N·s/m` is the dominant non-foil damping and is a tuning knob, not physics.
- Lateral offset 0.35 m with a 0.6 m half-span means the two "halves" overlap geometrically if they are meant to be adjacent panels from the centreline (centres at ±0.3 m). It is a lever-arm choice, not a layout check.
- Surface tension ($\gamma \approx 0.073\text{ N/m}$, capillary wavelength $\lambda_c \approx 1.7\text{ cm}$) is omitted from bulk hydrofoil/wave equations ($\text{We} \sim 3.4\times 10^4\text{–}2.5\times 10^5$ across operational speeds).
- Ventilation currently triggers on fixed geometric submergence (`vent_breach_hc = 0.25`) and isolated strut sideslip (`strut_vent_deg = 8.5°`), without pressure-driven $\text{Fr}_h$-scaled sub-surface seal rupture or strut-to-foil T-junction air propagation yet.

## Unity issues (obsolete — Unity removed on branch `mujoco-viewer`; kept as a record)
| # | Sev | Issue |
|---|---|---|
| U1 | — | ~~Unity auto-launch machine-specific path~~ — Unity removed. |
| U2 | M | `Floater.cs` logs every FixedUpdate when underwater (only when enabled). |
| U3 | M | `SimulinkTCPReceiver` uses `Thread.Abort` (unsupported on modern runtimes) and a byte-order TODO. |
| U4 | L | Both URP and HDRP packages/assets present. |
| U5 | L | Several dead scripts (`FoilData`, `HightBalancer`, `HydrofoilElement`, `HydrofoilController`). |

## Verification status (updated; fix pass in doc 14)
- After the fixes: 42/42 tests pass; before/after numbers in doc 14 §H.
- Executed since the first pass: `pytest` (23/23 pass), takeoff run, linearisation, bank/disturbance sweeps, RK4 convergence, Munk comparison. Results in doc 13.
- **Not reproduced:** the upstream fuzz-divergence claim and the Ki=0.05-diverges-in-waves claim (the latter cannot occur: waves never excite roll).
- Unity was removed; its scene wiring was never inspected.
