from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np


@dataclass
class Obstacle:
    x: float
    y: float
    vx: float
    vy: float
    r: float


class AutopilotGymEnv:
    def __init__(
        self,
        width: float = 800.0,
        height: float = 600.0,
        n_obstacles: int = 8,
        obstacle_radius: float = 10.0,
        obstacle_speed_min: float = 40.0,
        obstacle_speed_max: float = 120.0,
        dt: float = 0.1,
        max_steps: int = 800,
        seed: int | None = None,
    ) -> None:
        self.width = float(width)
        self.height = float(height)
        self.n_obstacles = int(n_obstacles)
        self.obstacle_radius = float(obstacle_radius)
        self.obstacle_speed_min = float(obstacle_speed_min)
        self.obstacle_speed_max = float(obstacle_speed_max)
        self.dt = float(dt)
        self.max_steps = int(max_steps)

        self._rng = random.Random(seed)

        self.agent_radius = 10.0
        self.agent_max_speed = 180.0
        self.agent_accel = 260.0

        self.goal_radius = 16.0

        self._step_n = 0
        self._ax = 0.0
        self._ay = 0.0
        self._avx = 0.0
        self._avy = 0.0
        self._gx = 0.0
        self._gy = 0.0
        self._obstacles: List[Obstacle] = []

    @property
    def obs_dim(self) -> int:
        return 6 + self.n_obstacles * 4

    @property
    def action_dim(self) -> int:
        return 5

    def reset(self) -> np.ndarray:
        self._step_n = 0

        self._ax, self._ay = self._sample_point(margin=40.0)
        self._avx, self._avy = 0.0, 0.0

        while True:
            self._gx, self._gy = self._sample_point(margin=40.0)
            if math.hypot(self._gx - self._ax, self._gy - self._ay) > min(self.width, self.height) * 0.5:
                break

        self._obstacles = []
        attempts = 0
        while len(self._obstacles) < self.n_obstacles and attempts < self.n_obstacles * 50:
            attempts += 1
            x, y = self._sample_point(margin=self.obstacle_radius + 10.0)

            if math.hypot(x - self._ax, y - self._ay) < 80.0:
                continue
            if math.hypot(x - self._gx, y - self._gy) < 80.0:
                continue

            ang = self._rng.uniform(0.0, math.tau)
            spd = self._rng.uniform(self.obstacle_speed_min, self.obstacle_speed_max)
            vx = math.cos(ang) * spd
            vy = math.sin(ang) * spd

            self._obstacles.append(Obstacle(x=x, y=y, vx=vx, vy=vy, r=self.obstacle_radius))

        return self._obs()

    def step(self, action: int) -> Tuple[np.ndarray, float, bool, dict]:
        self._step_n += 1

        ax_cmd, ay_cmd = 0.0, 0.0
        if action == 0:
            ax_cmd = -1.0
        elif action == 1:
            ax_cmd = 1.0
        elif action == 2:
            ay_cmd = -1.0
        elif action == 3:
            ay_cmd = 1.0

        self._avx += ax_cmd * self.agent_accel * self.dt
        self._avy += ay_cmd * self.agent_accel * self.dt

        spd = math.hypot(self._avx, self._avy)
        if spd > self.agent_max_speed:
            s = self.agent_max_speed / spd
            self._avx *= s
            self._avy *= s

        self._ax += self._avx * self.dt
        self._ay += self._avy * self.dt

        if self._ax < self.agent_radius:
            self._ax = self.agent_radius
            self._avx = 0.0
        if self._ax > self.width - self.agent_radius:
            self._ax = self.width - self.agent_radius
            self._avx = 0.0
        if self._ay < self.agent_radius:
            self._ay = self.agent_radius
            self._avy = 0.0
        if self._ay > self.height - self.agent_radius:
            self._ay = self.height - self.agent_radius
            self._avy = 0.0

        for o in self._obstacles:
            o.x += o.vx * self.dt
            o.y += o.vy * self.dt

            if o.x - o.r < 0.0 or o.x + o.r > self.width:
                o.vx *= -1.0
                o.x = max(o.r, min(o.x, self.width - o.r))
            if o.y - o.r < 0.0 or o.y + o.r > self.height:
                o.vy *= -1.0
                o.y = max(o.r, min(o.y, self.height - o.r))

        done = False
        reward = -0.01
        info: dict = {}

        if self._collides():
            reward = -100.0
            done = True
            info["terminal_reason"] = "collision"

        if not done and math.hypot(self._gx - self._ax, self._gy - self._ay) <= self.goal_radius:
            reward = 100.0
            done = True
            info["terminal_reason"] = "goal"

        if not done and self._step_n >= self.max_steps:
            done = True
            info["terminal_reason"] = "timeout"

        return self._obs(), float(reward), bool(done), info

    def _collides(self) -> bool:
        for o in self._obstacles:
            if math.hypot(o.x - self._ax, o.y - self._ay) <= (o.r + self.agent_radius):
                return True
        return False

    def _obs(self) -> np.ndarray:
        obs = np.zeros((self.obs_dim,), dtype=np.float32)

        obs[0] = self._ax
        obs[1] = self._ay
        obs[2] = self._avx
        obs[3] = self._avy
        obs[4] = self._gx
        obs[5] = self._gy

        k = 6
        for o in self._obstacles[: self.n_obstacles]:
            obs[k + 0] = o.x
            obs[k + 1] = o.y
            obs[k + 2] = o.vx
            obs[k + 3] = o.vy
            k += 4

        return obs

    def _sample_point(self, margin: float) -> Tuple[float, float]:
        x = self._rng.uniform(margin, self.width - margin)
        y = self._rng.uniform(margin, self.height - margin)
        return float(x), float(y)
