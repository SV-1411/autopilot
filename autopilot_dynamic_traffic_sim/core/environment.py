from __future__ import annotations

import csv
import math
import random
from typing import List, Tuple

import pygame

from config import (
    AGENT_COLOR,
    AGENT_HEADING_COLOR,
    AGENT_RADIUS,
    BACKGROUND_COLOR,
    CONTROLLER,
    CSV_EXPORT,
    CSV_FILENAME,
    OBSTACLE_COLOR,
    RL_GOAL_RADIUS,
    RL_MODEL_PATH,
    RL_OBSTACLE_COUNT,
    RL_OBSTACLE_SPEED_MAX,
    RL_OBSTACLE_SPEED_MIN,
    DENSE_MOVING_OBSTACLE_COUNT,
    DENSE_STATIC_FILL_PROB,
    DENSE_STATIC_MODE,
    DYNAMIC_BLOCK_PADDING_CELLS,
    DWA_TRAJ_COLOR,
    GOAL_COLOR,
    GRID_CELL_SIZE,
    MOVING_OBSTACLE_COUNT,
    MOVING_OBSTACLE_MAX_SPEED,
    MOVING_OBSTACLE_MIN_SPEED,
    MOVING_OBSTACLE_RADIUS,
    PATH_COLOR,
    REPLAN_INTERVAL,
    REPLAN_PREDICT_TIME,
    SCREEN_HEIGHT,
    SCREEN_WIDTH,
    STATIC_OBSTACLE_SAMPLE_RADIUS_CELLS,
    START_COLOR,
    PATH_SMOOTHING,
    PATH_SMOOTHING_MAX_SKIP,
    USE_DWA,
)
from core.agent import Agent
from core.obstacle import MovingObstacle
from planning.astar import astar
from planning.collision import circle_collision
from planning.grid_map import GridMap
from planning.path_smoothing import smooth_path


class Environment:
    def __init__(self) -> None:
        self.grid_map = GridMap()

        self.start_cell = (2, 2)
        self.goal_cell = (self.grid_map.rows - 3, self.grid_map.cols - 3)

        sx, sy = self.grid_map.grid_to_world(*self.start_cell)
        self.agent = Agent(sx, sy)

        self.obstacles: List[MovingObstacle] = []
        self.static_obstacles: List[Tuple[float, float, float]] = []  # (x, y, radius)
        self._blocked_snapshot: List[Tuple[int, int]] = []

        self._replan_timer = 0.0
        self._collisions = 0
        self._sim_time = 0.0

        self._dense_static_enabled = bool(DENSE_STATIC_MODE)
        self._dense_fill_prob = float(DENSE_STATIC_FILL_PROB)
        self._dense_moving_count = int(DENSE_MOVING_OBSTACLE_COUNT)
        self._normal_moving_count = int(MOVING_OBSTACLE_COUNT)
        self._csv_obstacle_slots = max(self._dense_moving_count, self._normal_moving_count)

        self._csv_file = None
        self._csv_writer = None
        if CSV_EXPORT:
            self._csv_file = open(CSV_FILENAME, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            header = ["t", "ax", "ay", "ayaw", "av", "aomega"]
            for i in range(self._csv_obstacle_slots):
                header.extend([f"ox{i}", f"oy{i}", f"ovx{i}", f"ovy{i}", f"or{i}"])
            self._csv_writer.writerow(header)

        self._rl_mode = CONTROLLER == "rl"
        if self._rl_mode:
            self._setup_rl()
        else:
            self._build_static_scenario()
            self._add_static_obstacles()
            self._spawn_moving_obstacles(self._moving_obstacle_target_count())
            self._plan_path()

    # ---------------------------------------------------------------- RL mode

    def _setup_rl(self) -> None:
        """Run a trained policy end-to-end, reusing the training env dynamics."""
        from rl.env import AutopilotGymEnv
        from core.controllers import RLController

        self._gym = AutopilotGymEnv(
            width=SCREEN_WIDTH,
            height=SCREEN_HEIGHT,
            n_obstacles=RL_OBSTACLE_COUNT,
            obstacle_speed_min=RL_OBSTACLE_SPEED_MIN,
            obstacle_speed_max=RL_OBSTACLE_SPEED_MAX,
            goal_radius=RL_GOAL_RADIUS,
        )
        self._rl = RLController(
            RL_MODEL_PATH, SCREEN_WIDTH, SCREEN_HEIGHT,
            self.agent._limits, RL_OBSTACLE_COUNT,
        )
        self._gym_obs, _ = self._gym.reset()
        self._rl_accum = 0.0
        self._rl_episodes = 0
        self._rl_goals = 0
        self._rl_collisions = 0

        if not pygame.font.get_init():
            pygame.font.init()
        self._font = pygame.font.SysFont(None, 22)

    def _update_rl(self, dt: float) -> None:
        # Step at the policy's native timestep so playback is real-time.
        self._rl_accum += dt
        guard = 0
        while self._rl_accum >= self._gym.dt and guard < 10:
            guard += 1
            self._rl_accum -= self._gym.dt
            action, _ = self._rl.model.predict(self._gym_obs, deterministic=True)
            self._gym_obs, _, terminated, truncated, info = self._gym.step(action)
            if terminated or truncated:
                reason = info.get("terminal_reason")
                self._rl_episodes += 1
                if reason == "goal":
                    self._rl_goals += 1
                elif reason == "collision":
                    self._rl_collisions += 1
                self._gym_obs, _ = self._gym.reset()

    def _render_rl(self, surface: pygame.Surface) -> None:
        surface.fill(BACKGROUND_COLOR)
        g = self._gym

        pygame.draw.circle(surface, GOAL_COLOR, (int(g._gx), int(g._gy)), int(g.goal_radius), 2)
        pygame.draw.circle(surface, GOAL_COLOR, (int(g._gx), int(g._gy)), 6)

        for o in g._obstacles:
            pygame.draw.circle(surface, OBSTACLE_COLOR, (int(o.x), int(o.y)), int(o.r))

        pygame.draw.circle(surface, AGENT_COLOR, (int(g._ax), int(g._ay)), int(g.agent_radius))
        hx = g._ax + math.cos(g._heading) * (g.agent_radius + 10)
        hy = g._ay + math.sin(g._heading) * (g.agent_radius + 10)
        pygame.draw.line(surface, AGENT_HEADING_COLOR, (g._ax, g._ay), (hx, hy), 2)

        rate = (self._rl_goals / self._rl_episodes * 100.0) if self._rl_episodes else 0.0
        lines = [
            "Controller: RL (trained policy)",
            f"Episodes: {self._rl_episodes}",
            f"Goals: {self._rl_goals}   Collisions: {self._rl_collisions}",
            f"Success: {rate:.0f}%",
        ]
        y = 6
        for ln in lines:
            surface.blit(self._font.render(ln, True, (230, 230, 230)), (8, y))
            y += 20

    def handle_event(self, event: pygame.event.Event) -> None:
        if getattr(self, "_rl_mode", False):
            return
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_o:
                self._spawn_moving_obstacles(self._moving_obstacle_target_count())
                self._replan_timer = REPLAN_INTERVAL
                return

            if event.key == pygame.K_g:
                self._dense_static_enabled = not self._dense_static_enabled
                self._build_static_scenario()
                self._spawn_moving_obstacles(self._moving_obstacle_target_count())
                self._replan_timer = REPLAN_INTERVAL
                return

            if event.key == pygame.K_r:
                self._build_static_scenario()
                self._spawn_moving_obstacles(self._moving_obstacle_target_count())
                self._replan_timer = REPLAN_INTERVAL
                return

            if event.key in (pygame.K_LEFTBRACKET, pygame.K_RIGHTBRACKET):
                delta = -0.05 if event.key == pygame.K_LEFTBRACKET else 0.05
                self._dense_fill_prob = max(0.05, min(0.85, self._dense_fill_prob + delta))
                if self._dense_static_enabled:
                    self._build_static_scenario()
                    self._replan_timer = REPLAN_INTERVAL
                return

            if event.key in (pygame.K_MINUS, pygame.K_EQUALS, pygame.K_KP_MINUS, pygame.K_KP_PLUS):
                inc = event.key in (pygame.K_EQUALS, pygame.K_KP_PLUS)
                self._dense_moving_count = max(0, min(self._csv_obstacle_slots, self._dense_moving_count + (2 if inc else -2)))
                self._normal_moving_count = max(0, min(self._csv_obstacle_slots, self._normal_moving_count + (2 if inc else -2)))
                self._spawn_moving_obstacles(self._moving_obstacle_target_count())
                self._replan_timer = REPLAN_INTERVAL
                return

        if event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = event.pos
            cell = self.grid_map.world_to_grid(float(mx), float(my))
            if not self.grid_map.in_bounds(*cell):
                return
            if cell == self.start_cell or cell == self.goal_cell:
                return

            mods = pygame.key.get_mods()
            if (mods & pygame.KMOD_SHIFT) and event.button == 1:
                self.goal_cell = cell
                self.grid_map.clear_cell(*self.goal_cell)
                self._replan_timer = REPLAN_INTERVAL
                return

            if event.button == 1:
                self.grid_map.set_obstacle(*cell)
                self._replan_timer = REPLAN_INTERVAL
                return

            if event.button == 3:
                self.grid_map.clear_cell(*cell)
                self._replan_timer = REPLAN_INTERVAL
                return

    def _add_static_obstacles(self) -> None:
        """No static obstacles - only moving obstacles"""
        self.static_obstacles = []

    def _build_static_scenario(self) -> None:
        self.grid_map.grid[:, :] = 0
        self.grid_map.set_border_walls()

        if self._dense_static_enabled:
            self._generate_dense_static(self._dense_fill_prob)
            return

        self.grid_map.set_rect_obstacle(top=6, left=6, height=10, width=2)
        self.grid_map.set_rect_obstacle(top=6, left=6, height=2, width=18)
        self.grid_map.set_rect_obstacle(top=14, left=12, height=2, width=18)
        self.grid_map.set_rect_obstacle(top=20, left=20, height=8, width=2)
        self.grid_map.set_rect_obstacle(top=28, left=10, height=2, width=16)

        self.grid_map.clear_cell(*self.start_cell)
        self.grid_map.clear_cell(*self.goal_cell)

    def _moving_obstacle_target_count(self) -> int:
        target = self._dense_moving_count if self._dense_static_enabled else self._normal_moving_count
        return int(max(0, min(self._csv_obstacle_slots, target)))

    def _generate_dense_static(self, fill_prob: float) -> None:
        prob = float(fill_prob)

        for _ in range(10):
            self.grid_map.grid[:, :] = 0
            self.grid_map.set_border_walls()

            for i in range(1, self.grid_map.rows - 1):
                for j in range(1, self.grid_map.cols - 1):
                    if (i, j) == self.start_cell or (i, j) == self.goal_cell:
                        continue
                    if random.random() < prob:
                        self.grid_map.set_obstacle(i, j)

            self.grid_map.clear_cell(*self.start_cell)
            self.grid_map.clear_cell(*self.goal_cell)

            test_path = astar(self.start_cell, self.goal_cell, self.grid_map)
            if test_path:
                return

            prob *= 0.85

        si, sj = self.start_cell
        gi, gj = self.goal_cell
        steps = max(abs(gi - si), abs(gj - sj))
        for k in range(steps + 1):
            i = int(round(si + (gi - si) * (k / steps)))
            j = int(round(sj + (gj - sj) * (k / steps)))
            self.grid_map.clear_cell(i, j)

    def _plan_path(self) -> None:
        agent_cell = self.grid_map.world_to_grid(self.agent.x, self.agent.y)
        start = agent_cell if self.grid_map.is_free(*agent_cell) else self.start_cell
        path_cells = astar(start, self.goal_cell, self.grid_map)
        if PATH_SMOOTHING and path_cells:
            path_cells = smooth_path(path_cells, self.grid_map, max_skip=int(PATH_SMOOTHING_MAX_SKIP))
        waypoints = [self.grid_map.grid_to_world(i, j) for (i, j) in path_cells]
        self.agent.set_path(waypoints)

    def _spawn_moving_obstacles(self, count: int) -> None:
        self.obstacles.clear()

        count = int(max(0, min(self._csv_obstacle_slots, count)))

        start_world = self.grid_map.grid_to_world(*self.start_cell)
        goal_world = self.grid_map.grid_to_world(*self.goal_cell)

        attempts = 0
        while len(self.obstacles) < count and attempts < count * 50:
            attempts += 1

            x = random.uniform(MOVING_OBSTACLE_RADIUS + 10, SCREEN_WIDTH - MOVING_OBSTACLE_RADIUS - 10)
            y = random.uniform(MOVING_OBSTACLE_RADIUS + 10, SCREEN_HEIGHT - MOVING_OBSTACLE_RADIUS - 10)

            cell = self.grid_map.world_to_grid(x, y)
            if not self.grid_map.is_free(*cell):
                continue

            if circle_collision(x, y, MOVING_OBSTACLE_RADIUS + 25, start_world[0], start_world[1], 0):
                continue
            if circle_collision(x, y, MOVING_OBSTACLE_RADIUS + 25, goal_world[0], goal_world[1], 0):
                continue

            angle = random.uniform(0.0, math.tau)
            # SLOW speeds for training
            speed = random.uniform(10.0, 30.0)  # Much slower than before
            vx = math.cos(angle) * speed
            vy = math.sin(angle) * speed

            self.obstacles.append(MovingObstacle(x=x, y=y, vx=vx, vy=vy, radius=MOVING_OBSTACLE_RADIUS))

    def _clear_dynamic_blocks(self) -> None:
        for i, j in self._blocked_snapshot:
            self.grid_map.clear_cell(i, j)
        self._blocked_snapshot.clear()

    def _apply_dynamic_blocks(self) -> None:
        self._clear_dynamic_blocks()

        blocked: set[Tuple[int, int]] = set()

        for obs in self.obstacles:
            px = obs.x + obs.vx * REPLAN_PREDICT_TIME
            py = obs.y + obs.vy * REPLAN_PREDICT_TIME

            ci, cj = self.grid_map.world_to_grid(px, py)
            for di in range(-DYNAMIC_BLOCK_PADDING_CELLS, DYNAMIC_BLOCK_PADDING_CELLS + 1):
                for dj in range(-DYNAMIC_BLOCK_PADDING_CELLS, DYNAMIC_BLOCK_PADDING_CELLS + 1):
                    ii = ci + di
                    jj = cj + dj
                    if not self.grid_map.in_bounds(ii, jj):
                        continue
                    if (ii, jj) == self.start_cell or (ii, jj) == self.goal_cell:
                        continue
                    blocked.add((ii, jj))

        for i, j in blocked:
            if self.grid_map.is_free(i, j):
                self.grid_map.set_obstacle(i, j)
                self._blocked_snapshot.append((i, j))

    def _collect_local_obstacles(self) -> List[Tuple[float, float, float]]:
        obstacles: List[Tuple[float, float, float]] = []

        # Only add moving obstacles (no static obstacles)
        for obs in self.obstacles:
            obstacles.append((obs.x, obs.y, obs.radius))

        ai, aj = self.grid_map.world_to_grid(self.agent.x, self.agent.y)
        r = STATIC_OBSTACLE_SAMPLE_RADIUS_CELLS
        static_radius = (GRID_CELL_SIZE * 0.5) * 0.95
        for i in range(ai - r, ai + r + 1):
            for j in range(aj - r, aj + r + 1):
                if not self.grid_map.in_bounds(i, j):
                    continue
                if self.grid_map.grid[i, j] != 1:
                    continue
                cx, cy = self.grid_map.grid_to_world(i, j)
                obstacles.append((cx, cy, static_radius))

        return obstacles

    def _reset_agent(self) -> None:
        sx, sy = self.grid_map.grid_to_world(*self.start_cell)
        self.agent.x = sx
        self.agent.y = sy
        self.agent.speed = 0.0
        self.agent.omega = 0.0

    def update(self, dt: float) -> None:
        if self._rl_mode:
            self._update_rl(dt)
            return

        # Update moving obstacles
        for obs in self.obstacles:
            obs.update(dt)

        if (
            self.agent.x < AGENT_RADIUS
            or self.agent.x > SCREEN_WIDTH - AGENT_RADIUS
            or self.agent.y < AGENT_RADIUS
            or self.agent.y > SCREEN_HEIGHT - AGENT_RADIUS
        ):
            self._reset_agent()
            self._replan_timer = REPLAN_INTERVAL

        # Check collisions with moving obstacles
        for obs in self.obstacles:
            if circle_collision(self.agent.x, self.agent.y, float(AGENT_RADIUS), obs.x, obs.y, obs.radius):
                self._collisions += 1
                self._reset_agent()
                self._replan_timer = REPLAN_INTERVAL
                break

        self._replan_timer += dt
        self._sim_time += dt
        if self._replan_timer >= REPLAN_INTERVAL:
            self._apply_dynamic_blocks()
            self._plan_path()
            self._clear_dynamic_blocks()
            self._replan_timer = 0.0

        local_obstacles = self._collect_local_obstacles() if USE_DWA else None
        self.agent.update(dt, obstacles=local_obstacles)

        if CSV_EXPORT and self._csv_writer:
            row = [round(self._sim_time, 3), round(self.agent.x, 2), round(self.agent.y, 2), round(self.agent.heading, 3), round(self.agent.speed, 2), round(getattr(self.agent, "omega", 0.0), 3)]
            for obs in self.obstacles[: self._csv_obstacle_slots]:
                row.extend([round(obs.x, 2), round(obs.y, 2), round(obs.vx, 2), round(obs.vy, 2), round(obs.radius, 2)])
            missing = self._csv_obstacle_slots - min(len(self.obstacles), self._csv_obstacle_slots)
            for _ in range(missing):
                row.extend(["", "", "", "", ""])
            self._csv_writer.writerow(row)

    def render(self, surface: pygame.Surface) -> None:
        if self._rl_mode:
            self._render_rl(surface)
            return

        surface.fill(BACKGROUND_COLOR)
        self.grid_map.draw(surface)

        path = self.agent.get_path()
        if len(path) >= 2:
            pygame.draw.lines(surface, PATH_COLOR, False, path, 3)

        start_pos = self.grid_map.grid_to_world(*self.start_cell)
        goal_pos = self.grid_map.grid_to_world(*self.goal_cell)

        pygame.draw.circle(surface, START_COLOR, (int(start_pos[0]), int(start_pos[1])), 8)
        pygame.draw.circle(surface, GOAL_COLOR, (int(goal_pos[0]), int(goal_pos[1])), 8)

        # No static obstacles to draw

        # Draw moving obstacles
        for obs in self.obstacles:
            obs.draw(surface)

        traj = self.agent.get_debug_trajectory()
        if USE_DWA and len(traj) >= 2:
            pygame.draw.lines(surface, DWA_TRAJ_COLOR, False, traj, 2)

        self.agent.draw(surface)

    def __del__(self) -> None:
        if self._csv_file:
            self._csv_file.close()
