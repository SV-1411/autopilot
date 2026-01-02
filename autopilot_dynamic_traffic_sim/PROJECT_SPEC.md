# PROJECT_SPEC

## Goal

Build a 2D autonomous navigation simulation where an agent (car/ship) moves from start to goal in a rectangular world with static and moving obstacles.

The codebase must stay modular:

- Simulation layer (Pygame loop, objects, time step)
- Planning layer (A*, replanning, DWA later)

## Repository layout

```
autopilot_dynamic_traffic_sim/
  main.py
  config.py
  core/
    environment.py
    agent.py
    obstacle.py
  planning/
    grid_map.py
    astar.py
    collision.py
  docs/
    design.md
    algorithms.md
    usage.md
```

## Public contracts (Phase 1)

- `planning.astar.astar(start, goal, grid_map) -> list[tuple[int,int]]`
- `planning.grid_map.GridMap.world_to_grid(x,y) -> (i,j)`
- `planning.grid_map.GridMap.grid_to_world(i,j) -> (x,y)`
- `core.environment.Environment.update(dt)`
- `core.environment.Environment.render(surface)`

## Phases

- Phase 1: Grid + static obstacles + A* + agent waypoint following
- Phase 2: Moving obstacles + collision checking + periodic replanning
- Phase 3: DWA/VO local planner
- Phase 4 (optional): RL in Colab using a gym-like interface
