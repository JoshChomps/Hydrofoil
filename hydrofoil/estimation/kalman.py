"""
Speed-scheduled steady-state Kalman filter on the linearised plant (hydrofoil/linearize.py).

State estimated: [z phi theta v w p q r] (surge u comes straight from the speed sensor). The model is the
foilborne plant linearised at trim for a few speeds, discretised at the control rate; the filter gain is the
steady-state (DARE) gain per speed, interpolated in the measured speed. Inputs are the ACHIEVED flap angles
relative to trim. Measurements are whatever arrived this step (z, phi, theta, p, q, r); missing sensors are
simply skipped. Latency is not compensated (the delayed height sample is treated as current).
"""

from __future__ import annotations

import numpy as np
from scipy.linalg import expm, solve_discrete_are

from hydrofoil.linearize import trim, linearize, reduced

STATE_IDX = [0, 1, 2, 4, 5, 6, 7, 8]                       # reduced [z phi theta u v w p q r] without u
MEAS_NAMES = ("z", "phi", "theta", "p", "q", "r")
MEAS_STATE = {"z": 0, "phi": 1, "theta": 2, "p": 5, "q": 6, "r": 7}


def _discretise(A, B, dt):
    n, m = B.shape
    M = np.zeros((n + m, n + m)); M[:n, :n] = A; M[:n, n:] = B
    E = expm(M * dt)
    return E[:n, :n], E[:n, n:]


class KalmanEstimator:
    def __init__(self, vp, dt: float, speeds=(3.0, 4.0, 5.0, 6.0, 7.0), sigmas: dict | None = None,
                 process: dict | None = None):
        d = np.radians
        s = {"z": 0.01, "phi": d(0.3), "theta": d(0.3), "p": d(0.2), "q": d(0.2), "r": d(0.2)}
        s.update(sigmas or {})
        # process noise std per sqrt(s) on the states (unmodelled forces / waves); PLACEHOLDER tuning
        q = {"z": 1e-4, "phi": 1e-4, "theta": 1e-4, "v": 0.3, "w": 0.15, "p": 0.5, "q": 0.5, "r": 0.3}
        q.update(process or {})
        self.dt, self.speeds = dt, np.asarray(speeds, float)
        Rv = np.diag([s[k] ** 2 for k in MEAS_NAMES])
        H = np.zeros((6, 8))
        for i, k in enumerate(MEAS_NAMES):
            H[i, MEAS_STATE[k]] = 1.0
        Qw = np.diag([q[k] ** 2 * dt for k in ("z", "phi", "theta", "v", "w", "p", "q", "r")])
        Ads, Bds, Ls, trims = [], [], [], []
        for sp in self.speeds:
            x0, u0 = trim(vp, sp)
            A, B = linearize(vp, x0, u0)
            Ar, Br, _ = reduced(A, B)
            Ad, Bd = _discretise(Ar[np.ix_(STATE_IDX, STATE_IDX)], Br[STATE_IDX, :], dt)
            P = solve_discrete_are(Ad.T, H.T, Qw, Rv)
            L = P @ H.T @ np.linalg.inv(H @ P @ H.T + Rv)
            Ads.append(Ad); Bds.append(Bd); Ls.append(L); trims.append(u0[:3])
        self.Ad, self.Bd, self.L, self.trim = map(np.asarray, (Ads, Bds, Ls, trims))
        self.H = H
        self.reset()

    def reset(self):
        self.x = np.zeros(8)
        self.z_ref0 = None

    def _interp(self, arr, u):
        u = float(np.clip(u, self.speeds[0], self.speeds[-1]))
        j = int(np.clip(np.searchsorted(self.speeds, u) - 1, 0, len(self.speeds) - 2))
        f = (u - self.speeds[j]) / (self.speeds[j + 1] - self.speeds[j])
        return (1 - f) * arr[j] + f * arr[j + 1]

    def init(self, meas: dict, z_trim: float):
        """Start from the first measurements (unmeasured v, w at 0). State is kept as deviation from level trim."""
        self.z_trim = z_trim
        for k in MEAS_NAMES:
            if k in meas:
                self.x[MEAS_STATE[k]] = meas[k] - (z_trim if k == "z" else 0.0)

    def update(self, meas: dict, u_cmd, u_speed: float, z_trim: float) -> np.ndarray:
        """One predict + update. u_cmd = achieved [main, elev, roll] deg. Returns estimate [z phi theta v w p q r]
        in absolute coordinates (z in NED, angles/rates in rad)."""
        Ad, Bd, L, T = (self._interp(a, u_speed) for a in (self.Ad, self.Bd, self.L, self.trim))
        self.x = Ad @ self.x + Bd @ (np.asarray(u_cmd, float) - T)
        y = np.array([meas.get(k, np.nan) - (z_trim if k == "z" else 0.0) for k in MEAS_NAMES])
        ok = ~np.isnan(y)
        if ok.any():
            innov = np.where(ok, y - self.H @ self.x, 0.0)
            self.x = self.x + (L * ok[None, :]) @ innov
        out = self.x.copy()
        out[0] += z_trim
        return out
