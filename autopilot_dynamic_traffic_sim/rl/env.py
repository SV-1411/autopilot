"""Gymnasium environment for autopilot navigation.

A headless (no Pygame) env suitable for Colab / GPU training with
Stable-Baselines3. It uses the SAME unicycle dynamics (``core.dynamics``) and
the SAME observation builder (``rl.obs``) as the live simulation, so a policy
trained here can be deployed back into ``main.py`` without behavioral drift.

Action space (continuous, deployable to the sim's unicycle agent):
    a[0] = throttle  in [-1, 1]  -> accelerate / brake forward speed
    a[1] = yaw       in [-1, 1]  -> desired yaw rate

Observation: see ``rl.obs.build_observation``.

Reward:
    + progress toward the goal (potential-based shaping)  -- the key signal
    - small per-step time penalty
    + goal bonus (terminated)
    - collision penalty (terminated)
    timeout -> truncated
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np

try:
    import gymnasium as gym
    from gymnasium import spaces
except ModuleNotFoundError as exc:  # pragma: no cover - helpful message in Colab
    raise ModuleNotFoundError(
        "gymnasium is required for rl.env. Install training deps with "
        "`pip install -r rl/requirements-train.txt`."
    ) from exc

from core.dynamics import KinematicLimits, step_unicycle
from rl.obs import build_observation, observation_dim


@dataclass
class Obstacle:
    x: float
    y: float
    vx: float
    vy: float
    r: float


class AutopilotGymEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        width: float = 800.0,
        height: float = 600.0,
        n_obstacles: int = 8,
        obstacle_radius: float = 10.0,
        obstacle_speed_min: float = 20.0,
        obstacle_speed_max: float = 60.0,
        dt: float = 0.1,
        max_steps: int = 800,
        # Agent kinematics (match the sim's DWA_CONFIG so policies transfer).
        agent_radius: float = 10.0,
        max_speed: float = 180.0,
        min_speed: float = 0.0,
        max_yaw_rate: float = 2.5,
        max_accel: float = 260.0,
        goal_radius: float = 16.0,
        # Reward shaping.
        progress_gain: float = 0.05,
        time_penalty: float = 0.01,
        goal_reward: float = 100.0,
        collision_penalty: float = 100.0,
        # Dense obstacle-avoidance shaping: penalize getting within
        # ``safe_margin`` px of an obstacle surface, scaled by how deep in.
        clearance_gain: float = 0.0,
        safe_margin: float = 30.0,
        render_mode: Optional[str] = None,
    ) -> None:
        super().__init__()

        self.width = float(width)
        self.height = float(height)
        self.n_obstacles = int(n_obstacles)
        self.obstacle_radius = float(obstacle_radius)
        self.obstacle_speed_min = float(obstacle_speed_min)
        self.obstacle_speed_max = float(obstacle_speed_max)
        self.dt = float(dt)
        self.max_steps = int(max_steps)

        self.agent_radius = float(agent_radius)
        self._limits = KinematicLimits(
            max_speed=float(max_speed),
            min_speed=float(min_speed),
            max_yaw_rate=float(max_yaw_rate),
            max_accel=float(max_accel),
        )
        self.goal_radius = float(goal_radius)

        self.progress_gain = float(progress_gain)
        self.time_penalty = float(time_penalty)
        self.goal_reward = float(goal_reward)
        self.collision_penalty = float(collision_penalty)
        self.clearance_gain = float(clearance_gain)
        self.safe_margin = float(safe_margin)
        self.render_mode = render_mode

        self._rng = random.Random()

        # Agent state: (x, y, heading, speed, omega)
        self._ax = 0.0
        self._ay = 0.0
        self._heading = 0.0
        self._speed = 0.0
        self._omega = 0.0

        self._gx = 0.0
        self._gy = 0.0
        self._obstacles: List[Obstacle] = []

        self._step_n = 0
        self._prev_goal_dist = 0.0

        obs_dim = observation_dim(self.n_obstacles)
        self.observation_space = spaces.Box(
            low=-1.0, high=1.0, shape=(obs_dim,), dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(2,), dtype=np.float32
        )

    # ------------------------------------------------------------------ API

    def set_difficulty(
        self,
        obstacle_speed_min: Optional[float] = None,
        obstacle_speed_max: Optional[float] = None,
        goal_radius: Optional[float] = None,
    ) -> None:
        """Adjust difficulty at runtime (used by the training curriculum).

        Only obstacle *speed* and goal radius change here; the obstacle *count*
        is fixed because it determines the observation size.
        Takes effect on the next ``reset()``.
        """
        if obstacle_speed_min is not None:
            self.obstacle_speed_min = float(obstacle_speed_min)
        if obstacle_speed_max is not None:
            self.obstacle_speed_max = float(obstacle_speed_max)
        if goal_radius is not None:
            self.goal_radius = float(goal_radius)

    def get_difficulty(self) -> dict:
        return {
            "obstacle_speed_min": self.obstacle_speed_min,
            "obstacle_speed_max": self.obstacle_speed_max,
            "goal_radius": self.goal_radius,
        }

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        if seed is not None:
            self._rng = random.Random(seed)

        self._step_n = 0

        self._ax, self._ay = self._sample_point(margin=40.0)
        self._heading = self._rng.uniform(-math.pi, math.pi)
        self._speed = 0.0
        self._omega = 0.0

        while True:
            self._gx, self._gy = self._sample_point(margin=40.0)
            if (
                math.hypot(self._gx - self._ax, self._gy - self._ay)
                > min(self.width, self.height) * 0.5
            ):
                break

        self._spawn_obstacles()

        self._prev_goal_dist = math.hypot(self._gx - self._ax, self._gy - self._ay)
        return self._obs(), {}

    def step(self, action: np.ndarray):
        self._step_n += 1

        throttle = float(np.clip(action[0], -1.0, 1.0))
        yaw = float(np.clip(action[1], -1.0, 1.0))

        v_cmd = self._speed + throttle * self._limits.max_accel * self.dt
        omega_cmd = yaw * self._limits.max_yaw_rate

        state = (self._ax, self._ay, self._heading, self._speed, self._omega)
        self._ax, self._ay, self._heading, self._speed, self._omega = step_unicycle(
            state, v_cmd, omega_cmd, self.dt, self._limits
        )

        self._clamp_agent_to_bounds()
        self._update_obstacles()

        goal_dist = math.hypot(self._gx - self._ax, self._gy - self._ay)

        # Potential-based shaping: reward shrinking the distance to the goal.
        progress = self._prev_goal_dist - goal_dist
        reward = self.progress_gain * progress - self.time_penalty
        self._prev_goal_dist = goal_dist

        # Dense avoidance: penalize crowding an obstacle so the agent learns to
        # keep margin (veer early) instead of only fearing the terminal hit.
        if self.clearance_gain > 0.0:
            clearance = self._nearest_clearance()
            if clearance < self.safe_margin:
                reward -= self.clearance_gain * (
                    (self.safe_margin - clearance) / self.safe_margin
                )

        terminated = False
        truncated = False
        info: dict = {}

        if self._collides():
            reward = -self.collision_penalty
            terminated = True
            info["terminal_reason"] = "collision"
        elif goal_dist <= self.goal_radius:
            reward = self.goal_reward
            terminated = True
            info["terminal_reason"] = "goal"
        elif self._step_n >= self.max_steps:
            truncated = True
            info["terminal_reason"] = "timeout"

        info["is_success"] = info.get("terminal_reason") == "goal"
        return self._obs(), float(reward), terminated, truncated, info

    # -------------------------------------------------------------- internals

    def _obs(self) -> np.ndarray:
        obstacles = [(o.x, o.y, o.vx, o.vy) for o in self._obstacles]
        return build_observation(
            ax=self._ax,
            ay=self._ay,
            heading=self._heading,
            speed=self._speed,
            omega=self._omega,
            gx=self._gx,
            gy=self._gy,
            obstacles=obstacles,
            world_w=self.width,
            world_h=self.height,
            max_speed=self._limits.max_speed,
            max_yaw_rate=self._limits.max_yaw_rate,
            n_obstacles=self.n_obstacles,
        )

    def _spawn_obstacles(self) -> None:
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
            self._obstacles.append(
                Obstacle(
                    x=x,
                    y=y,
                    vx=math.cos(ang) * spd,
                    vy=math.sin(ang) * spd,
                    r=self.obstacle_radius,
                )
            )

    def _update_obstacles(self) -> None:
        for o in self._obstacles:
            o.x += o.vx * self.dt
            o.y += o.vy * self.dt

            if o.x - o.r < 0.0 or o.x + o.r > self.width:
                o.vx *= -1.0
                o.x = max(o.r, min(o.x, self.width - o.r))
            if o.y - o.r < 0.0 or o.y + o.r > self.height:
                o.vy *= -1.0
                o.y = max(o.r, min(o.y, self.height - o.r))

    def _clamp_agent_to_bounds(self) -> None:
        if self._ax < self.agent_radius:
            self._ax = self.agent_radius
            self._speed = 0.0
        elif self._ax > self.width - self.agent_radius:
            self._ax = self.width - self.agent_radius
            self._speed = 0.0
        if self._ay < self.agent_radius:
            self._ay = self.agent_radius
            self._speed = 0.0
        elif self._ay > self.height - self.agent_radius:
            self._ay = self.height - self.agent_radius
            self._speed = 0.0

    def _collides(self) -> bool:
        for o in self._obstacles:
            if math.hypot(o.x - self._ax, o.y - self._ay) <= (o.r + self.agent_radius):
                return True
        return False

    def _nearest_clearance(self) -> float:
        """Distance (px) from the agent's edge to the nearest obstacle edge."""
        best = float("inf")
        for o in self._obstacles:
            d = math.hypot(o.x - self._ax, o.y - self._ay) - (o.r + self.agent_radius)
            if d < best:
                best = d
        return best

    def _sample_point(self, margin: float) -> Tuple[float, float]:
        x = self._rng.uniform(margin, self.width - margin)
        y = self._rng.uniform(margin, self.height - margin)
        return float(x), float(y)
