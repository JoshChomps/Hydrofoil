# Hydrofoil Sim — Documentation

Written 2026-11-04 as a record of the repo at commit `2af5157`
(<https://github.com/jacobleathem/hydrofoil-sim>), by reading every script.
Docs 01–12 were written by reading; doc 13 and the updates it triggered were produced by running the code
(see [11-known-issues.md](11-known-issues.md) → "Verification status").

| Doc | What's in it |
|-----|--------------|
| [01-overview-and-status.md](01-overview-and-status.md) | What the project is, what exists, progress, what's stub/missing |
| [02-code-reference.md](02-code-reference.md) | Every file and class/function, one by one |
| [03-math-and-physics.md](03-math-and-physics.md) | Equations of motion, foil model, hull, waves, integration |
| [04-constants-and-parameters.md](04-constants-and-parameters.md) | Every constant, default parameter, gain, with units and provenance |
| [05-constraints-and-limits.md](05-constraints-and-limits.md) | Clamps, saturation limits, stability limits, modelling assumptions |
| [06-control-system.md](06-control-system.md) | PID loops, actuator model, surge loop, sign conventions |
| [07-interfaces.md](07-interfaces.md) | UDP packet formats, REST/WebSocket API, CLI flags |
| [08-mujoco-viewer.md](08-mujoco-viewer.md) | The MuJoCo app (replaced Unity and the dashboard): sliders, plots, drive read-out, frames, keys |
| [09-manual.md](09-manual.md) | How to install, run, operate, and tune |
| [10-roadmap.md](10-roadmap.md) | Planned and suggested future development |
| [11-known-issues.md](11-known-issues.md) | Bugs, inconsistencies, stale docs, unverified items |
| [12-progress-log.md](12-progress-log.md) | Dated record of project history and this documentation pass |
| [13-math-audit-and-pid-critique.md](13-math-audit-and-pid-critique.md) | Math audit as found: PID method and flaws, physics omissions (pre-fix state) |
| [18-audit-verification.md](18-audit-verification.md) | **Every audit claim re-checked against the original code**: verdicts, evidence, corrections to my own claims |
| [17-actuators-moteus-drive.md](17-actuators-moteus-drive.md) | Control-surface drives: hinge moments, moteus/BLDC/gearbox current-voltage-power budget, gear-ratio sweep, stress, failure modes, next steps |
| [15-buoyancy-hydrostatics-cross-reference.md](15-buoyancy-hydrostatics-cross-reference.md) | Hull buoyancy checked against metacentric theory; distributed hydrostatics model; before/after stiffness |
| [16-review-guide.md](16-review-guide.md) | **Start here for review:** conventions, equations→code map, assumptions ledger, evidence/commands, open questions |
| [14-error-register-and-fixes.md](14-error-register-and-fixes.md) | **Every error expanded: cause, evidence, fix, test; retuned gains; unit/conversion map; still-open items** |

Conventions used across the code (and these docs): Fossen (2011) notation,
NED world frame (z **down**), body frame x-forward / y-starboard / z-down,
ZYX Euler angles in radians internally, degrees at UI/UDP-input boundaries.
