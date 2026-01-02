from __future__ import annotations

import math
from typing import Iterable, List, Tuple

Point = Tuple[float, float]


def circle_collision(x1: float, y1: float, r1: float, x2: float, y2: float, r2: float) -> bool:
    return math.hypot(x2 - x1, y2 - y1) <= (r1 + r2)


def segment_intersects_circle(p1: Point, p2: Point, center: Point, radius: float) -> bool:
    x1, y1 = p1
    x2, y2 = p2
    cx, cy = center

    dx = x2 - x1
    dy = y2 - y1
    if dx == 0.0 and dy == 0.0:
        return math.hypot(cx - x1, cy - y1) <= radius

    t = ((cx - x1) * dx + (cy - y1) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))

    closest_x = x1 + t * dx
    closest_y = y1 + t * dy

    return math.hypot(cx - closest_x, cy - closest_y) <= radius


def is_path_collision_free(
    waypoints: List[Point],
    obstacles: Iterable[Tuple[float, float, float]],
    agent_radius: float,
) -> bool:
    if len(waypoints) < 2:
        return True

    for a, b in zip(waypoints, waypoints[1:]):
        for ox, oy, oradius in obstacles:
            if segment_intersects_circle(a, b, (ox, oy), agent_radius + oradius):
                return False

    return True
