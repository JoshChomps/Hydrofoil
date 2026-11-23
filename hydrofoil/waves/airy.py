"""
Linear (Airy) wave model.

Provides wave surface elevation and orbital velocity at a given
(x, z, t) position. The orbital velocity perturbs the relative
velocity seen by each foil, which is how waves produce disturbance forces.

Reference: Fossen (2011) Appendix D; Newman (1977) Marine Hydrodynamics.
"""

from __future__ import annotations

import numpy as np

from hydrofoil.constants import RHO_WATER, G


def encounter_frequency(
    omega_rad_s: float,
    speed_m_s: float,
    heading_rel_rad: float,
    g: float = G,
) -> float:
    """
    Deep-water wave encounter angular frequency omega_e [rad/s]:

        omega_e = omega - (omega^2 * U / g) * cos(mu)

    where mu = wave_direction - boat_heading (travel direction convention:
    0 = following sea, +/- pi/2 = beam sea, pi = head sea).
    """
    om = float(omega_rad_s)
    return float(om - (om * om * float(speed_m_s) / float(g)) * float(np.cos(heading_rel_rad)))


class AiryWave:
    """
    A single regular (monochromatic) wave.

    Parameters
    ----------
    amplitude   : wave amplitude (half height) [m]
    wavelength  : spatial wavelength [m]
    period      : wave period [s]. Default None = derive from the deep-water
                  dispersion relation omega^2 = g k, so wavelength and period
                  cannot disagree. Passing an explicit value overrides it
                  (non-physical combinations are then the caller's problem).
    direction   : wave travel direction in NED x-y plane [rad], 0 = +x
    phase       : initial phase offset [rad]
    rho         : water density [kg/m³]
    g           : gravitational acceleration [m/s²]
    """

    def __init__(
        self,
        amplitude: float = 0.05,
        wavelength: float = 5.0,
        period: float | None = None,
        direction: float = 0.0,
        phase: float = 0.0,
        rho: float = RHO_WATER,
        g: float = G,
    ):
        if wavelength <= 0.0:
            raise ValueError(f"wavelength must be > 0, got {wavelength}")
        if period is not None and period <= 0.0:
            raise ValueError(f"period must be > 0, got {period}")
        self.A = amplitude
        self.lam = wavelength
        self.k = 2 * np.pi / wavelength     # wavenumber [rad/m]
        self.T = (2 * np.pi / np.sqrt(g * self.k)) if period is None else period
        self.direction = direction
        self.phase = phase
        self.rho = rho
        self.g = g

        self.omega = 2 * np.pi / self.T     # angular frequency [rad/s]

    @property
    def amplitude(self) -> float:
        """Equivalent regular amplitude / half significant wave height [m]."""
        return float(self.A)

    def elevation(self, x: float, y: float, t: float) -> float:
        """Surface elevation eta_w(x, y, t) [m] relative to still water."""
        kx = self.k * (x * np.cos(self.direction) + y * np.sin(self.direction))
        return self.A * np.cos(kx - self.omega * t + self.phase)

    def orbital_velocity(self, x: float, y: float, z: float, t: float) -> np.ndarray:
        """
        Wave particle orbital velocity [u_w, v_w, w_w] in NED frame [m/s].

        Valid for z <= 0 (below still water surface). Decays exponentially
        with depth via e^(k*z) for deep water (k*depth >> 1).

        Parameters
        ----------
        x, y : horizontal NED position [m]
        z    : NED depth, negative downward from surface [m]
        t    : time [s]

        Returns
        -------
        V_wave : (3,) NED orbital velocity [m/s]
        """
        kx = self.k * (x * np.cos(self.direction) + y * np.sin(self.direction))
        phase_arg = kx - self.omega * t + self.phase
        # Linear Airy theory is valid for z <= 0; clamp above still-water datum so
        # emerged foils or wave-crest samples cannot cause exponential blowup.
        z_eff = min(float(z), 0.0)
        depth_decay = np.exp(self.k * z_eff)

        Vh = self.A * self.omega * depth_decay * np.cos(phase_arg)
        Vv = self.A * self.omega * depth_decay * np.sin(phase_arg)

        V_wave = np.array([
            Vh * np.cos(self.direction),
            Vh * np.sin(self.direction),
            -Vv,   # positive upward = negative NED z
        ])
        return V_wave


class CalmWater(AiryWave):
    """No-wave environment — zero elevation and velocity everywhere."""

    def __init__(self):
        super().__init__(amplitude=0.0, wavelength=1.0)

    def elevation(self, x, y, t):
        return 0.0

    def orbital_velocity(self, x, y, z, t):
        return np.zeros(3)


class IrregularWave(AiryWave):
    """
    Multi-component irregular JONSWAP directional sea state using linear
    superposition with optional cos^(2s) directional spreading.

    Matches the exact AiryWave interface (`elevation(x, y, t)` supporting scalar
    or array inputs, `orbital_velocity(x, y, z, t)`, and `.amplitude = Hs / 2`).
    """

    def __init__(
        self,
        significant_height: float = 0.20,
        peak_wavelength: float = 12.0,
        peak_period: float | None = None,
        direction: float = 0.0,
        gamma: float = 3.3,
        spread_s: float | None = 4.0,
        n_freq: int = 24,
        n_dir: int = 5,
        seed: int = 42,
        rho: float = RHO_WATER,
        g: float = G,
    ):
        hs = max(float(significant_height), 0.0)
        super().__init__(
            amplitude=0.5 * hs,
            wavelength=float(peak_wavelength),
            period=peak_period,
            direction=float(direction),
            rho=rho,
            g=g,
        )
        self.Hs = hs
        self.gamma = max(float(gamma), 1.0)
        self.spread_s = spread_s
        nf = max(int(n_freq), 1)
        nd = max(int(n_dir), 1) if (spread_s is not None and float(spread_s) > 0.0) else 1
        rng = np.random.default_rng(int(seed))

        om_p = float(self.omega)
        if nf == 1:
            freqs = np.array([om_p], dtype=float)
            s_vals = np.array([1.0], dtype=float)
        else:
            edges = np.linspace(0.55 * om_p, 2.60 * om_p, nf + 1)
            df = np.diff(edges)
            # Stratified random frequency within each bin prevents exact periodic repetition
            freqs = edges[:-1] + rng.uniform(0.15, 0.85, size=nf) * df
            sig = np.where(freqs <= om_p, 0.07, 0.09)
            r_exp = np.exp(-((freqs / om_p - 1.0) ** 2) / (2.0 * sig ** 2))
            s0 = (freqs ** -5) * np.exp(-1.25 * (om_p / freqs) ** 4) * (self.gamma ** r_exp)
            s_vals = s0 * df

        if nd == 1:
            dirs = np.array([float(direction)], dtype=float)
            w_dir = np.array([1.0], dtype=float)
        else:
            dth = np.linspace(-np.pi / 3.0, np.pi / 3.0, nd)
            dirs = float(direction) + dth
            raw_w = np.maximum(np.cos(dth), 1e-6) ** (2.0 * float(spread_s))
            w_dir = raw_w / np.sum(raw_w)

        e_grid = np.outer(s_vals, w_dir)
        e_sum = float(np.sum(e_grid))
        m0 = (0.25 * hs) ** 2
        if e_sum > 0.0 and m0 > 0.0:
            amp_grid = np.sqrt(2.0 * m0 * e_grid / e_sum)
        else:
            amp_grid = np.zeros_like(e_grid)

        om_grid, dir_grid = np.meshgrid(freqs, dirs, indexing="ij")
        self._amp = np.ascontiguousarray(amp_grid.ravel(), dtype=float)
        self._omega = np.ascontiguousarray(om_grid.ravel(), dtype=float)
        self._k = np.ascontiguousarray((self._omega ** 2) / float(g), dtype=float)
        self._cos_t = np.ascontiguousarray(np.cos(dir_grid.ravel()), dtype=float)
        self._sin_t = np.ascontiguousarray(np.sin(dir_grid.ravel()), dtype=float)
        self._kx = np.ascontiguousarray(self._k * self._cos_t, dtype=float)
        self._ky = np.ascontiguousarray(self._k * self._sin_t, dtype=float)
        self._phase = np.ascontiguousarray(rng.uniform(0.0, 2.0 * np.pi, size=self._amp.size), dtype=float)

    def elevation(self, x: float, y: float, t: float) -> float:
        if np.ndim(x) == 0 and np.ndim(y) == 0:
            arg = self._kx * float(x) + self._ky * float(y) - self._omega * float(t) + self._phase
            return float(np.dot(self._amp, np.cos(arg)))
        xa = np.asarray(x, dtype=float)
        ya = np.asarray(y, dtype=float)
        arg = xa[..., None] * self._kx + ya[..., None] * self._ky - self._omega * float(t) + self._phase
        return np.cos(arg) @ self._amp

    def orbital_velocity(self, x: float, y: float, z: float, t: float) -> np.ndarray:
        z_eff = min(float(z), 0.0)
        arg = self._kx * float(x) + self._ky * float(y) - self._omega * float(t) + self._phase
        dec = self._amp * self._omega * np.exp(self._k * z_eff)
        vh_c = dec * np.cos(arg)
        return np.array([
            float(np.dot(vh_c, self._cos_t)),
            float(np.dot(vh_c, self._sin_t)),
            -float(np.dot(dec, np.sin(arg))),
        ])
