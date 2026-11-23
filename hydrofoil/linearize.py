"""
Trim solver and linearisation of the foilborne plant (docs/10 Phase 3).

State x = [x y z phi theta psi | u v w p q r]  (NED position, Euler angles, body velocities).
Inputs  u = [delta_main, delta_elev, delta_roll, thrust]  (flap angles in degrees, thrust in N).

The plant linearised here is `VehiclePhysics.physics_rhs` in calm water, i.e. WITHOUT the controllers
and the surface-drive dynamics. Wing->tail downwash is included quasi-steadily (no transport lag; the lag
is a hidden delay state and is left out of A, B).

    vp = VehiclePhysics(params)
    xt, ut = trim(vp, speed=4.0)               # level flight at the configured flying height
    A, B = linearize(vp, xt, ut)               # 12 x 12, 12 x 4
    Ar, Br, idx = reduced(A, B)                # states [z phi theta u v w p q r], inputs [main elev roll]
"""

from __future__ import annotations

import numpy as np

from hydrofoil.vehicle import VehiclePhysics

# state indices kept in the reduced model (x, y, psi do not feed back into the dynamics)
REDUCED_STATES = [2, 3, 4, 6, 7, 8, 9, 10, 11]
REDUCED_NAMES = ["z", "phi", "theta", "u", "v", "w", "p", "q", "r"]
INPUT_NAMES = ["delta_main", "delta_elev", "delta_roll", "thrust"]


def _rhs(vp: VehiclePhysics, x, u) -> np.ndarray:
    """physics_rhs with the quasi-steady downwash consistent with the evaluated point."""
    cmds = {"delta_main": u[0], "delta_elev": u[1], "delta_roll": u[2]}
    vp._eps_used = 0.0
    vp.physics_rhs(x, cmds, u[3])                 # populates the wing's last_CL
    vp._eps_used = vp._downwash_qs()
    return vp.physics_rhs(x, cmds, u[3])


def level_state(vp: VehiclePhysics, speed: float, theta: float = 0.0, flying_height: float | None = None):
    p = vp.params
    h = p["flying_height"] if flying_height is None else flying_height
    x = np.zeros(12)
    x[2] = float(getattr(vp, "hull_offset", p["hull_offset"])) - h
    x[4] = theta
    x[6] = speed
    x[8] = speed * np.tan(theta)      # level flight path: z-dot = -u sin(theta) + w cos(theta) = 0
    return x


def trim(vp: VehiclePhysics, speed: float, theta: float = 0.0, flying_height: float | None = None,
         u0=(15.0, 0.0, 0.0, 15.0), tol: float = 1e-9, max_iter: int = 40):
    """
    Level foilborne trim at `speed`: solves du/dt = dw/dt = dq/dt = 0 for (delta_main, delta_elev, thrust)
    with phi = v = p = r = 0, delta_roll = 0 and the pitch angle fixed at `theta` [rad].
    Returns (x_trim, u_trim). Raises RuntimeError if Newton does not converge.
    """
    x = level_state(vp, speed, theta, flying_height)
    free = [0, 1, 3]                      # delta_main, delta_elev, thrust
    rows = [6, 8, 10]                     # du, dw, dq

    def res(uu):
        return _rhs(vp, x, uu)[rows]

    starts = [u0, (0.0, 0.0, 0.0, 150.0), (-3.0, 0.0, 0.0, 250.0)]
    best_residual = float("inf")
    for s_init in starts:
        u = np.array(s_init, dtype=float)
        for _ in range(max_iter):
            r = res(u)
            if np.max(np.abs(r)) < tol:
                return x, u
            J = np.zeros((3, 3))
            for j, k in enumerate(free):
                e = np.zeros(4); e[k] = 1e-4
                J[:, j] = (res(u + e) - res(u - e)) / 2e-4
            try:
                step = np.linalg.solve(J, -r)
            except np.linalg.LinAlgError:
                step = np.linalg.lstsq(J.T @ J + 1e-4 * np.eye(J.shape[1]), -J.T @ r, rcond=None)[0]
            u[free] += np.clip(step, [-5.0, -5.0, -200.0], [5.0, 5.0, 200.0])   # deg, deg, N
        best_residual = min(best_residual, float(np.max(np.abs(res(u)))))
    raise RuntimeError(f"trim did not converge at {speed} m/s (residual {best_residual:.3g})")


def linearize(vp: VehiclePhysics, x0, u0, dx: float = 1e-6, du: float = 1e-4):
    """Central-difference Jacobians A = df/dx (12x12) and B = df/du (12x4) at (x0, u0)."""
    x0, u0 = np.asarray(x0, float), np.asarray(u0, float)
    A = np.zeros((12, 12))
    B = np.zeros((12, 4))
    for i in range(12):
        e = np.zeros(12); e[i] = dx
        A[:, i] = (_rhs(vp, x0 + e, u0) - _rhs(vp, x0 - e, u0)) / (2 * dx)
    for j in range(4):
        e = np.zeros(4); e[j] = du
        B[:, j] = (_rhs(vp, x0, u0 + e) - _rhs(vp, x0, u0 - e)) / (2 * du)
    return A, B


def reduced(A, B):
    """Drop x, y, psi; keep flap inputs only. Returns (Ar 9x9, Br 9x3, state indices)."""
    idx = REDUCED_STATES
    return A[np.ix_(idx, idx)], B[np.ix_(idx, [0, 1, 2])], idx
