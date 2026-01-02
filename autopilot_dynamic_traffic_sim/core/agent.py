from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple

import pygame

from config import (
    AGENT_COLOR,
    AGENT_HEADING_COLOR,
    AGENT_MAX_SPEED,
    AGENT_RADIUS,
    DWA_CONFIG,
    USE_DWA,
    WAYPOINT_TOLERANCE_PX,
)
from planning.dwa import config_from_dict, dwa_step


class Agent:
    def __init__(self, x: float, y: float, heading: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.heading = heading
        self.speed = 0.0
        self.omega = 0.0

        self._dwa_cfg = config_from_dict(DWA_CONFIG)
        self._debug_traj: List[Tuple[float, float]] = []

        self._path: List[Tuple[float, float]] = []
        self._path_index = 0

    def set_path(self, waypoints: Sequence[Tuple[float, float]]) -> None:
        self._path = list(waypoints)
        self._path_index = 0

    def get_path(self) -> List[Tuple[float, float]]:
        return list(self._path)

    def get_path_index(self) -> int:
        return self._path_index

    def get_debug_trajectory(self) -> List[Tuple[float, float]]:
        return list(self._debug_traj)

    def update(
        self,
        dt: float,
        obstacles: Optional[Iterable[Tuple[float, float, float]]] = None,
        goal: Optional[Tuple[float, float]] = None,
    ) -> None:
        if not self._path or self._path_index >= len(self._path):
            self.speed = 0.0
            self.omega = 0.0
            self._debug_traj = [(self.x, self.y)]
            return

        target_x, target_y = self._path[self._path_index]
        dx = target_x - self.x
        dy = target_y - self.y
        dist = math.hypot(dx, dy)

        if dist <= WAYPOINT_TOLERANCE_PX:
            self._path_index += 1
            return

        if USE_DWA and obstacles is not None:
            local_goal = (target_x, target_y)
            state = [self.x, self.y, self.heading, self.speed, self.omega]
            v_cmd, omega_cmd, traj = dwa_step(
                state=state,
                cfg=self._dwa_cfg,
                obstacles=obstacles,
                goal=local_goal,
                robot_radius=float(AGENT_RADIUS),
            )
            self._debug_traj = traj

            self.speed = max(self._dwa_cfg.min_speed, min(self._dwa_cfg.max_speed, v_cmd))
            self.omega = max(-self._dwa_cfg.max_yaw_rate, min(self._dwa_cfg.max_yaw_rate, omega_cmd))

            self.heading += self.omega * dt
            self.heading = ((self.heading + math.pi) % (2.0 * math.pi)) - math.pi

            self.x += math.cos(self.heading) * self.speed * dt
            self.y += math.sin(self.heading) * self.speed * dt
            return

        self.heading = math.atan2(dy, dx)
        self.speed = AGENT_MAX_SPEED
        self.omega = 0.0
        self._debug_traj = [(self.x, self.y), (target_x, target_y)]

        step = self.speed * dt
        if step > dist:
            step = dist

        self.x += math.cos(self.heading) * step
        self.y += math.sin(self.heading) * step

    def draw(self, surface: pygame.Surface) -> None:
        pygame.draw.circle(surface, AGENT_COLOR, (int(self.x), int(self.y)), AGENT_RADIUS)

        hx = self.x + math.cos(self.heading) * (AGENT_RADIUS + 10)
        hy = self.y + math.sin(self.heading) * (AGENT_RADIUS + 10)
        pygame.draw.line(surface, AGENT_HEADING_COLOR, (self.x, self.y), (hx, hy), 2)
