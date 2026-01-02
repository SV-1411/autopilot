from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

Point = Tuple[float, float]
Obstacle = Tuple[float, float, float]


@dataclass(frozen=True)
class DWAConfig:
    max_speed: float
    min_speed: float
    max_yaw_rate: float
    max_accel: float
    max_delta_yaw_rate: float
    v_resolution: float
    yaw_rate_resolution: float
    dt: float
    predict_time: float
    to_goal_cost_gain: float
    speed_cost_gain: float
    obstacle_cost_gain: float


def config_from_dict(d: dict) -> DWAConfig:
    return DWAConfig(
        max_speed=float(d["max_speed"]),
        min_speed=float(d["min_speed"]),
        max_yaw_rate=float(d["max_yaw_rate"]),
        max_accel=float(d["max_accel"]),
        max_delta_yaw_rate=float(d["max_delta_yaw_rate"]),
        v_resolution=float(d["v_resolution"]),
        yaw_rate_resolution=float(d["yaw_rate_resolution"]),
        dt=float(d["dt"]),
        predict_time=float(d["predict_time"]),
        to_goal_cost_gain=float(d["to_goal_cost_gain"]),
        speed_cost_gain=float(d["speed_cost_gain"]),
        obstacle_cost_gain=float(d["obstacle_cost_gain"]),
    )


def dwa_step(
    state: Sequence[float],
    cfg: DWAConfig,
    obstacles: Iterable[Obstacle],
    goal: Point,
    robot_radius: float,
) -> Tuple[float, float, List[Point]]:
    x, y, yaw, v, omega = float(state[0]), float(state[1]), float(state[2]), float(state[3]), float(state[4])

    dw = _calc_dynamic_window(v, omega, cfg)

    best_cost = float("inf")
    best_v = 0.0
    best_omega = 0.0
    best_traj: List[Point] = [(x, y)]

    v_min, v_max, w_min, w_max = dw

    v_candidate = v_min
    while v_candidate <= v_max + 1e-6:
        w_candidate = w_min
        while w_candidate <= w_max + 1e-6:
            traj = _predict_trajectory(x, y, yaw, v_candidate, w_candidate, cfg)
            cost = _calc_cost(traj, v_candidate, cfg, obstacles, goal, robot_radius)

            if cost < best_cost:
                best_cost = cost
                best_v = v_candidate
                best_omega = w_candidate
                best_traj = traj

            w_candidate += cfg.yaw_rate_resolution
        v_candidate += cfg.v_resolution

    return best_v, best_omega, best_traj


def _calc_dynamic_window(v: float, omega: float, cfg: DWAConfig) -> Tuple[float, float, float, float]:
    v_min = max(cfg.min_speed, v - cfg.max_accel * cfg.dt)
    v_max = min(cfg.max_speed, v + cfg.max_accel * cfg.dt)

    w_min = max(-cfg.max_yaw_rate, omega - cfg.max_delta_yaw_rate * cfg.dt)
    w_max = min(cfg.max_yaw_rate, omega + cfg.max_delta_yaw_rate * cfg.dt)

    return v_min, v_max, w_min, w_max


def _predict_trajectory(
    x: float,
    y: float,
    yaw: float,
    v: float,
    omega: float,
    cfg: DWAConfig,
) -> List[Point]:
    traj: List[Point] = [(x, y)]

    t = 0.0
    cx, cy, cyaw = x, y, yaw
    while t <= cfg.predict_time + 1e-6:
        cx, cy, cyaw = _motion(cx, cy, cyaw, v, omega, cfg.dt)
        traj.append((cx, cy))
        t += cfg.dt

    return traj


def _motion(x: float, y: float, yaw: float, v: float, omega: float, dt: float) -> Tuple[float, float, float]:
    yaw = _wrap_angle(yaw + omega * dt)
    x += math.cos(yaw) * v * dt
    y += math.sin(yaw) * v * dt
    return x, y, yaw


def _calc_cost(
    traj: List[Point],
    v: float,
    cfg: DWAConfig,
    obstacles: Iterable[Obstacle],
    goal: Point,
    robot_radius: float,
) -> float:
    goal_cost = cfg.to_goal_cost_gain * _dist(traj[-1], goal)
    speed_cost = cfg.speed_cost_gain * (cfg.max_speed - v)

    min_clear = _min_clearance(traj, obstacles, robot_radius)
    if min_clear <= 0.0:
        return float("inf")

    obstacle_cost = cfg.obstacle_cost_gain * (1.0 / (min_clear + 1e-6))

    return goal_cost + speed_cost + obstacle_cost


def _min_clearance(traj: List[Point], obstacles: Iterable[Obstacle], robot_radius: float) -> float:
    min_clear = float("inf")
    for px, py in traj:
        for ox, oy, r in obstacles:
            d = math.hypot(px - ox, py - oy) - (robot_radius + r)
            if d < min_clear:
                min_clear = d
                if min_clear <= 0.0:
                    return min_clear
    return min_clear


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
