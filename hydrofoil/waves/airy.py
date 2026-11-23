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
