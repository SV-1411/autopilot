from __future__ import annotations

from dataclasses import dataclass

import pygame

from config import OBSTACLE_COLOR, SCREEN_HEIGHT, SCREEN_WIDTH


@dataclass
class StaticObstacle:
    x: float
    y: float
    radius: float

    def update(self, dt: float) -> None:
        return

    def draw(self, surface: pygame.Surface) -> None:
        pygame.draw.circle(surface, OBSTACLE_COLOR, (int(self.x), int(self.y)), int(self.radius))


@dataclass
class MovingObstacle:
    x: float
    y: float
    vx: float
    vy: float
    radius: float

    def update(self, dt: float) -> None:
        self.x += self.vx * dt
        self.y += self.vy * dt

        if self.x - self.radius < 0 or self.x + self.radius > SCREEN_WIDTH:
            self.vx *= -1.0
            self.x = max(self.radius, min(self.x, SCREEN_WIDTH - self.radius))

        if self.y - self.radius < 0 or self.y + self.radius > SCREEN_HEIGHT:
            self.vy *= -1.0
            self.y = max(self.radius, min(self.y, SCREEN_HEIGHT - self.radius))

    def draw(self, surface: pygame.Surface) -> None:
        pygame.draw.circle(surface, OBSTACLE_COLOR, (int(self.x), int(self.y)), int(self.radius))
