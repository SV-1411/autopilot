# Design

## Problem

Navigate an agent from start to goal in a 2D world with obstacles.

## Architecture

- `main.py`
  - owns the Pygame loop
  - calls `Environment.update(dt)` and `Environment.render(surface)`

- `core/`
  - `Environment`: orchestrates scenario, planning calls, updates, rendering
  - `Agent`: kinematic agent + behavior (Phase 1 path following; later DWA)
  - `Obstacle`: static/moving obstacle models (moving used in later phases)

- `planning/`
  - `GridMap`: grid representation + world<->grid conversion
  - `A*`: global planner
  - `collision`: utilities for collision tests (used more in Phase 2+)

## Data flow

- Environment builds a scenario in `GridMap`
- Environment calls `astar()` to get a grid path
- Grid cells are converted to world-space waypoints
- Agent follows waypoints each frame
