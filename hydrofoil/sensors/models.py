"""
Sensor models (docs/10 "State estimation and sensors"). All values are PLACEHOLDERS until real parts are chosen.

Each sensor takes the true 12-state vector and returns a measurement, with white noise, a slowly drifting
bias where it matters, a sample rate (zero-order hold between samples), a pure latency, and dropouts.
`SensorSuite.measure(state, t)` returns the latest values as a dict; None means "no valid sample yet".

    gyro      p q r [rad/s]            200 Hz, noise 0.2 deg/s, bias random walk
    ahrs      phi theta [rad]          100 Hz, noise 0.3 deg   (stands in for a fused attitude solution)
    height    z [m]                    20 Hz, noise 1 cm, 50 ms latency, 2 % dropout. A ranging sensor (ultrasonic /
                                       ToF / radar) sees the LOCAL water surface, so in waves this is the CG depth
                                       relative to the wave surface under the boat (z + wave elevation), not relative
                                       to still water. Tilt of the beam is taken as corrected with the attitude.
    accel     az [m/s^2]               200 Hz, noise 0.05 m/s^2, bias random walk. Vertical (NED, down-positive)
                                       acceleration of the CG, i.e. the IMU's specific force rotated to vertical with
                                       gravity removed.
    speed     u [m/s]                  10 Hz, noise 0.1 m/s    (GPS / paddlewheel)
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Channel:
    names: tuple[str, ...]
    index: tuple[int, ...]            # entries of the 12-state vector this channel measures
    rate_hz: float
    sigma: float | tuple[float, ...]
    latency_s: float = 0.0
    dropout: float = 0.0
    bias_walk: float = 0.0            # bias random-walk std per sqrt(s)

    def __post_init__(self):
        self._sig = np.broadcast_to(np.asarray(self.sigma, float), (len(self.index),)).copy()
        self._bias = np.zeros(len(self.index))
        self._queue: deque = deque()
        self._next = 0.0
        self.value = None
        self.fresh = False                # True on the step a new sample was delivered (value is held otherwise)

    def reset(self):
        self._bias[:] = 0.0; self._queue.clear(); self._next = 0.0; self.value = None; self.fresh = False

    def step(self, state, t: float, dt: float, rng):
        self._bias += self.bias_walk * np.sqrt(dt) * rng.standard_normal(len(self._bias))
        if t + 1e-12 >= self._next:
            self._next += 1.0 / self.rate_hz
            if rng.random() >= self.dropout:
                z = np.asarray(state)[list(self.index)] + self._bias + self._sig * rng.standard_normal(len(self._sig))
                self._queue.append((t + self.latency_s, z))
        self.fresh = False
        while self._queue and self._queue[0][0] <= t + 1e-12:
            self.value = self._queue.popleft()[1]
            self.fresh = True
        return self.value


class SensorSuite:
    def __init__(self, seed: int = 0, scale: float = 1.0):
        d = np.radians
        self.rng = np.random.default_rng(seed)
        self.channels = {
            "gyro": Channel(("p", "q", "r"), (9, 10, 11), 200.0, scale * d(0.2), bias_walk=scale * d(0.01)),
            "ahrs": Channel(("phi", "theta"), (3, 4), 100.0, scale * d(0.3)),
            "height": Channel(("z",), (12,), 20.0, scale * 0.01, latency_s=0.05, dropout=0.02),
            "accel": Channel(("az",), (13,), 200.0, scale * 0.05, bias_walk=scale * 0.002),
            "speed": Channel(("u",), (6,), 10.0, scale * 0.1),
        }
        self.fresh: set[str] = set()      # names that got a NEW sample on the last measure() call
        self._vz_prev = None

    def reset(self):
        for c in self.channels.values():
            c.reset()
        self.fresh = set(); self._vz_prev = None

    def measure(self, state, t: float, dt: float, surface_elev: float = 0.0) -> dict:
        """Latest value of every channel (held between samples); `self.fresh` names the ones that are new this
        call. surface_elev = wave elevation under the height sensor [m, up-positive]."""
        state = np.asarray(state, float)
        phi, th = state[3], state[4]
        vz = -state[6] * np.sin(th) + state[7] * np.cos(th) * np.sin(phi) + state[8] * np.cos(th) * np.cos(phi)
        az = 0.0 if self._vz_prev is None else (vz - self._vz_prev) / dt
        self._vz_prev = vz
        ext = np.concatenate([state, [state[2] + surface_elev, az]])
        out, self.fresh = {}, set()
        for key, ch in self.channels.items():
            v = ch.step(ext, t, dt, self.rng)
            if v is not None:
                out.update(dict(zip(ch.names, v)))
                if ch.fresh:
                    self.fresh.update(ch.names)
        return out
