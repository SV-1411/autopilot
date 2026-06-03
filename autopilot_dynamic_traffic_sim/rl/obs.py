"""Shared observation builder (egocentric).

The RL environment (during training) and the deployed ``RLController`` (during
simulation) MUST construct observations identically, or a trained policy will
misbehave when plugged back into the sim. That shared logic lives here.

Everything is expressed in the **agent's own frame** (egocentric): "how far
ahead / to my left is the goal", "which way should I turn". This is the simple,
rotation-invariant information a controller actually needs, so the policy does
not have to learn a world->body coordinate transform first. All values are
normalized to roughly [-1, 1].

Layout, length ``5 + 5 * n_obstacles``:

    [0] speed / max_speed
    [1] omega / max_yaw_rate
    [2] sin(goal_bearing)      # +1 = goal directly to the left
    [3] cos(goal_bearing)      # +1 = goal straight ahead
    [4] goal_distance / world_diagonal
    then, for each of the ``n_obstacles`` nearest obstacles (closest first;
    missing slots zero-padded), in the agent's frame:
        forward / diag, left / diag,            # relative position
        rel_v_forward / max_speed, rel_v_left / max_speed,  # relative velocity
        closing_speed / (2*max_speed)           # +ve = approaching (threat)
"""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np

# Each obstacle is (x, y, vx, vy) in world coordinates. Static obstacles use
# vx = vy = 0.
Obstacle = Tuple[float, float, float, float]


def observation_dim(n_obstacles: int) -> int:
    return 5 + 5 * int(n_obstacles)


def build_observation(
    ax: float,
    ay: float,
    heading: float,
    speed: float,
    omega: float,
    gx: float,
    gy: float,
    obstacles: Iterable[Obstacle],
    world_w: float,
    world_h: float,
    max_speed: float,
    max_yaw_rate: float,
    n_obstacles: int,
) -> np.ndarray:
    obs = np.zeros((observation_dim(n_obstacles),), dtype=np.float32)

    diag = math.hypot(world_w, world_h)
    cos_h = math.cos(heading)
    sin_h = math.sin(heading)

    obs[0] = speed / max_speed if max_speed else 0.0
    obs[1] = omega / max_yaw_rate if max_yaw_rate else 0.0

    # Goal in the agent's frame.
    gfwd, gleft = _to_body(gx - ax, gy - ay, cos_h, sin_h)
    gdist = math.hypot(gfwd, gleft)
    if gdist > 1e-6:
        obs[2] = gleft / gdist          # sin(bearing)
        obs[3] = gfwd / gdist           # cos(bearing)
    else:
        obs[3] = 1.0                    # on top of goal: treat as "ahead"
    obs[4] = gdist / diag if diag else 0.0

    # Agent's own velocity vector (world frame), for closing-speed calc.
    avx = speed * cos_h
    avy = speed * sin_h

    # Nearest obstacles first, each rotated into the agent's frame.
    nearest = sorted(
        obstacles, key=lambda o: math.hypot(o[0] - ax, o[1] - ay)
    )[: int(n_obstacles)]

    k = 5
    for ox, oy, ovx, ovy in nearest:
        rx, ry = ox - ax, oy - ay
        dist = math.hypot(rx, ry)
        ofwd, oleft = _to_body(rx, ry, cos_h, sin_h)
        vfwd, vleft = _to_body(ovx, ovy, cos_h, sin_h)
        obs[k + 0] = ofwd / diag
        obs[k + 1] = oleft / diag
        obs[k + 2] = vfwd / max_speed if max_speed else 0.0
        obs[k + 3] = vleft / max_speed if max_speed else 0.0
        # Closing speed: rate at which the gap is shrinking (relative velocity
        # projected onto the agent->obstacle line). +ve = approaching.
        if dist > 1e-6 and max_speed:
            closing = -((rx * (ovx - avx) + ry * (ovy - avy)) / dist)
            obs[k + 4] = closing / (2.0 * max_speed)
        k += 5

    np.clip(obs, -1.0, 1.0, out=obs)
    return obs


def _to_body(rx: float, ry: float, cos_h: float, sin_h: float) -> Tuple[float, float]:
    """Rotate a world-frame vector into the agent's frame.

    Returns (forward, left): the component along the heading and the component
    to the agent's left.
    """
    forward = rx * cos_h + ry * sin_h
    left = -rx * sin_h + ry * cos_h
    return forward, left
