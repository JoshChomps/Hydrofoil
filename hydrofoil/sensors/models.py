"""
Sensor models (docs/10 "State estimation and sensors"). All values are PLACEHOLDERS until real parts are chosen.

Each sensor takes the true 12-state vector and returns a measurement, with white noise, a slowly drifting
bias where it matters, a sample rate (zero-order hold between samples), a pure latency, and dropouts.
`SensorSuite.measure(state, t)` returns the latest values as a dict; None means "no valid sample yet".

    gyro      p q r [rad/s]            200 Hz, noise 0.2 deg/s, bias random walk
    ahrs      phi theta [rad]          100 Hz, noise 0.3 deg   (stands in for a fused attitude solution)
    height    z [m, NED depth of the CG]  20 Hz, noise 1 cm, 50 ms latency, 2 % dropout
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

    def reset(self):
        self._bias[:] = 0.0; self._queue.clear(); self._next = 0.0; self.value = None

    def step(self, state, t: float, dt: float, rng):
        self._bias += self.bias_walk * np.sqrt(dt) * rng.standard_normal(len(self._bias))
        if t + 1e-12 >= self._next:
            self._next += 1.0 / self.rate_hz
            if rng.random() >= self.dropout:
                z = np.asarray(state)[list(self.index)] + self._bias + self._sig * rng.standard_normal(len(self._sig))
                self._queue.append((t + self.latency_s, z))
        while self._queue and self._queue[0][0] <= t + 1e-12:
            self.value = self._queue.popleft()[1]
        return self.value


class SensorSuite:
    def __init__(self, seed: int = 0, scale: float = 1.0):
        d = np.radians
        self.rng = np.random.default_rng(seed)
        self.channels = {
            "gyro": Channel(("p", "q", "r"), (9, 10, 11), 200.0, scale * d(0.2), bias_walk=scale * d(0.01)),
            "ahrs": Channel(("phi", "theta"), (3, 4), 100.0, scale * d(0.3)),
            "height": Channel(("z",), (2,), 20.0, scale * 0.01, latency_s=0.05, dropout=0.02),
            "speed": Channel(("u",), (6,), 10.0, scale * 0.1),
        }

    def reset(self):
        for c in self.channels.values():
            c.reset()

    def measure(self, state, t: float, dt: float) -> dict:
        out = {}
        for key, ch in self.channels.items():
            v = ch.step(state, t, dt, self.rng)
            if v is not None:
                out.update(dict(zip(ch.names, v)))
        return out
