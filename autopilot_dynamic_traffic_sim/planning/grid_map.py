from __future__ import annotations

from typing import Iterable, Tuple

import numpy as np
import pygame

from config import (
    GRID_CELL_SIZE,
    GRID_LINE_COLOR,
    OBSTACLE_COLOR,
    SCREEN_HEIGHT,
    SCREEN_WIDTH,
)


class GridMap:
    def __init__(self) -> None:
        self.cell_size = GRID_CELL_SIZE
        self.cols = SCREEN_WIDTH // self.cell_size
        self.rows = SCREEN_HEIGHT // self.cell_size
        self.grid = np.zeros((self.rows, self.cols), dtype=np.int8)

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        j = int(x // self.cell_size)
        i = int(y // self.cell_size)
        return i, j

    def grid_to_world(self, i: int, j: int) -> Tuple[float, float]:
        x = j * self.cell_size + self.cell_size / 2
        y = i * self.cell_size + self.cell_size / 2
        return x, y

    def in_bounds(self, i: int, j: int) -> bool:
        return 0 <= i < self.rows and 0 <= j < self.cols

    def is_free(self, i: int, j: int) -> bool:
        return self.in_bounds(i, j) and self.grid[i, j] == 0

    def set_obstacle(self, i: int, j: int) -> None:
        if self.in_bounds(i, j):
            self.grid[i, j] = 1

    def clear_cell(self, i: int, j: int) -> None:
        if self.in_bounds(i, j):
            self.grid[i, j] = 0

    def set_rect_obstacle(self, top: int, left: int, height: int, width: int) -> None:
        bottom = top + height
        right = left + width
        top = max(0, top)
        left = max(0, left)
        bottom = min(self.rows, bottom)
        right = min(self.cols, right)
        if top >= bottom or left >= right:
            return
        self.grid[top:bottom, left:right] = 1

    def set_border_walls(self) -> None:
        self.grid[0, :] = 1
        self.grid[self.rows - 1, :] = 1
        self.grid[:, 0] = 1
        self.grid[:, self.cols - 1] = 1

    def iter_obstacle_cells(self) -> Iterable[Tuple[int, int]]:
        obs = np.argwhere(self.grid == 1)
        for i, j in obs:
            yield int(i), int(j)

    def draw(self, surface: pygame.Surface) -> None:
        for i in range(self.rows + 1):
            y = i * self.cell_size
            pygame.draw.line(surface, GRID_LINE_COLOR, (0, y), (SCREEN_WIDTH, y), 1)

        for j in range(self.cols + 1):
            x = j * self.cell_size
            pygame.draw.line(surface, GRID_LINE_COLOR, (x, 0), (x, SCREEN_HEIGHT), 1)

        for i, j in self.iter_obstacle_cells():
            rect = pygame.Rect(j * self.cell_size, i * self.cell_size, self.cell_size, self.cell_size)
            pygame.draw.rect(surface, OBSTACLE_COLOR, rect)
