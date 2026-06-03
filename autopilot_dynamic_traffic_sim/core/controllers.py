"""Controllers that turn (state, goal, obstacles) into motion commands.

- ``DWAController`` wraps the existing dynamic-window planner (waypoint following).
- ``RLController`` runs a trained Stable-Baselines3 policy end-to-end. It builds
  the observation with the SAME ``rl.obs.build_observation`` used in training,
  so behavior matches the learning environment.
"""

from __future__ import annotations

from typing import Iterable, Sequence, Tuple

from core.dynamics import KinematicLimits
from rl.obs import build_observation


class RLController:
    """Wraps a trained PPO policy. ``predict_command`` returns (v_cmd, omega_cmd)."""

    def __init__(
        self,
        model_path: str,
        world_w: float,
        world_h: float,
        limits: KinematicLimits,
        n_obstacles: int,
    ) -> None:
        try:
            from stable_baselines3 import PPO
        except ModuleNotFoundError as exc:  # pragma: no cover
            raise ModuleNotFoundError(
                "stable-baselines3 is required for the RL controller. Install "
                "with `pip install -r rl/requirements-train.txt`."
            ) from exc

        self.model = PPO.load(model_path)
        self.world_w = float(world_w)
        self.world_h = float(world_h)
        self.limits = limits
        self.n_obstacles = int(n_obstacles)

    def predict_command(
        self,
        state: Sequence[float],
        goal: Tuple[float, float],
        obstacles: Iterable[Tuple[float, float, float, float]],
        dt: float,
    ) -> Tuple[float, float]:
        x, y, heading, speed, omega = (
            float(state[0]), float(state[1]), float(state[2]),
            float(state[3]), float(state[4]),
        )
        obs = build_observation(
            ax=x, ay=y, heading=heading, speed=speed, omega=omega,
            gx=float(goal[0]), gy=float(goal[1]),
            obstacles=obstacles,
            world_w=self.world_w, world_h=self.world_h,
            max_speed=self.limits.max_speed, max_yaw_rate=self.limits.max_yaw_rate,
            n_obstacles=self.n_obstacles,
        )
        action, _ = self.model.predict(obs, deterministic=True)
        throttle = float(action[0])
        yaw = float(action[1])

        # Same action mapping as rl.env.step.
        v_cmd = speed + throttle * self.limits.max_accel * dt
        omega_cmd = yaw * self.limits.max_yaw_rate
        return v_cmd, omega_cmd
