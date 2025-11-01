# controllers/debug_first_order.py
from __future__ import annotations
from typing import Tuple
import math

Pose = Tuple[float, float, float]

class FirstOrderTracker:
    def __init__(self, dt: float = 0.02, tau_pos: float = 0.30, tau_yaw: float = 0.25):
        self.dt = dt
        self.tau_pos = tau_pos
        self.tau_yaw = tau_yaw

    @staticmethod
    def _wrap(a: float) -> float:
        while a > math.pi:  a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def step(self, state: Pose, target: Pose) -> Pose:
        x, y, th = state
        tx, ty, tth = target
        a_pos = 1.0 - math.exp(-self.dt / max(1e-6, self.tau_pos))
        a_yaw = 1.0 - math.exp(-self.dt / max(1e-6, self.tau_yaw))
        x += a_pos * (tx - x)
        y += a_pos * (ty - y)
        th = self._wrap(th + a_yaw * self._wrap(tth - th))
        return (x, y, th)
