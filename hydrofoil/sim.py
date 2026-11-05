"""
Headless batch simulation (quick plots / trim checks).

Now a thin wrapper over vehicle.VehiclePhysics -- the SAME physics, controller
gains, actuator model, hull and foil corrections as the dashboard and viewer.
(It used to carry its own simplified copy of the model, which silently drifted:
unit flap effectiveness, pinned speed, different gains.)

Quick start:
    python -m hydrofoil.sim
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path

from hydrofoil.params import DEFAULT_PARAMS
from hydrofoil.vehicle import VehiclePhysics
from hydrofoil.waves.airy import CalmWater

T_END = 90.0   # long enough for takeoff AND height settling (see docs/13)


def run_simulation(wave=None, open_loop=False, params=None, t_end=T_END):
    """Closed-loop (or zero-flap open-loop) run from rest; returns a log dict of arrays."""
    p = dict(DEFAULT_PARAMS if params is None else params)
    wave = wave or CalmWater()
    vp = VehiclePhysics(p)
    state = vp.rest_state()
    z_ref = p["hull_offset"] - p["flying_height"]
    zero = {"delta_main": 0.0, "delta_elev": 0.0, "delta_roll": 0.0}
    log = {k: [] for k in ("t", "z", "phi", "theta", "d_main", "d_elev", "d_roll", "u", "draft")}
    t = 0.0
    while t <= t_end:
        state, cmds, thrust, draft, _ = vp.step(state, t, wave, cmds_override=zero if open_loop else None)
        log["t"].append(t); log["z"].append(state[2])
        log["phi"].append(np.degrees(state[3])); log["theta"].append(np.degrees(state[4]))
        log["d_main"].append(cmds["delta_main"]); log["d_elev"].append(cmds["delta_elev"])
        log["d_roll"].append(cmds["delta_roll"]); log["u"].append(state[6]); log["draft"].append(draft)
        t += p["dt"]
    out = {k: np.array(v) for k, v in log.items()}
    out["z_ref"] = z_ref
    return out


def plot_results(log, title="Hydrofoil Simulation"):
    LOG_ZREF = log["z_ref"]
    fig = plt.figure(figsize=(12, 8))
    fig.suptitle(title)
    gs = gridspec.GridSpec(3, 2, figure=fig)

    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(log['t'], log['z'])
    ax1.axhline(LOG_ZREF, color='r', linestyle='--', label='reference')
    ax1.set_ylabel('Depth z [m] (NED)')
    ax1.set_title('Heave (altitude)')
    ax1.legend()
    ax1.grid(True)

    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(log['t'], log['phi'])
    ax2.axhline(0, color='r', linestyle='--')
    ax2.set_ylabel('Roll φ [deg]')
    ax2.set_title('Roll')
    ax2.grid(True)

    ax3 = fig.add_subplot(gs[2, 0])
    ax3.plot(log['t'], log['theta'])
    ax3.axhline(0, color='r', linestyle='--')
    ax3.set_ylabel('Pitch θ [deg]')
    ax3.set_xlabel('Time [s]')
    ax3.set_title('Pitch')
    ax3.grid(True)

    ax4 = fig.add_subplot(gs[0, 1])
    ax4.plot(log['t'], log['d_main'], label='main foil')
    ax4.set_ylabel('Delta [deg]')
    ax4.set_title('Main foil command')
    ax4.legend()
    ax4.grid(True)

    ax5 = fig.add_subplot(gs[1, 1])
    ax5.plot(log['t'], log['d_elev'], color='orange', label='elevator')
    ax5.set_ylabel('Delta [deg]')
    ax5.set_title('Elevator command')
    ax5.legend()
    ax5.grid(True)

    ax6 = fig.add_subplot(gs[2, 1])
    ax6.plot(log['t'], log['d_roll'], color='green', label='roll diff')
    ax6.set_ylabel('Delta [deg]')
    ax6.set_xlabel('Time [s]')
    ax6.set_title('Roll differential command')
    ax6.legend()
    ax6.grid(True)

    plt.tight_layout()

    # Save to file; also attempt interactive display if a backend supports it
    slug = title.lower().replace(' ', '_').replace('—', '').replace('(', '').replace(')', '').strip('_')
    out = Path(__file__).parent.parent / "data" / f"{slug}.png"
    fig.savefig(out, dpi=120)
    print(f"Plot saved → {out}")

    try:
        plt.show()
    except Exception:
        pass
    plt.close(fig)


if __name__ == '__main__':
    from hydrofoil.viz import dashboard, animation_2d

    print("=== CLOSED-LOOP (PID controller) ===")
    log_cl = run_simulation()

    # Static PNG
    plot_results(log_cl, "T-Foil — Closed Loop (PID)")

    # Interactive time-series dashboard → opens in browser
    dashboard(log_cl, title="T-Foil — Closed Loop (PID)", z_ref=log_cl['z_ref'])

    # Animated 2D side-view → opens in browser
    animation_2d(log_cl, z_ref=log_cl['z_ref'], title="T-Foil — Closed Loop Side View")

    print("\n=== OPEN-LOOP (no controller — should diverge) ===")
    log_ol = run_simulation(open_loop=True)
    plot_results(log_ol, "T-Foil — Open Loop (should diverge)")
    dashboard(log_ol, title="T-Foil — Open Loop (diverges)", z_ref=log_cl['z_ref'])
    animation_2d(log_ol, z_ref=log_ol['z_ref'], title="T-Foil — Open Loop Side View")
