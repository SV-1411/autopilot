"""Shared kinematic model.

Both the live simulation (``core.agent.Agent``) and the RL training
environment (``rl.env``) integrate motion through ``step_unicycle`` so that a
policy trained in Colab behaves identically when deployed back into the sim.
Keep this module dependency-free (stdlib only) so it can be imported anywhere.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

# (x, y, heading, speed, omega)
State = Tuple[float, float, float, float, float]


@dataclass(frozen=True)
class KinematicLimits:
    max_speed: float
    min_speed: float
    max_yaw_rate: float
    max_accel: float


def wrap_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return ((angle + math.pi) % (2.0 * math.pi)) - math.pi


def step_unicycle(
    state: State,
    v_cmd: float,
    omega_cmd: float,
    dt: float,
    limits: KinematicLimits,
) -> State:
    """Advance a unicycle one step.

    ``v_cmd`` is the desired forward speed and ``omega_cmd`` the desired yaw
    rate; both are clamped to ``limits`` before integration. Returns the new
    ``(x, y, heading, speed, omega)`` tuple.
    """
    x, y, heading, _speed, _omega = state

    speed = min(limits.max_speed, max(limits.min_speed, v_cmd))
    omega = min(limits.max_yaw_rate, max(-limits.max_yaw_rate, omega_cmd))

    heading = wrap_angle(heading + omega * dt)
    x += math.cos(heading) * speed * dt
    y += math.sin(heading) * speed * dt

    return x, y, heading, speed, omega
