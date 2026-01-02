# Algorithms

## A* (Phase 1)

- Search space: 2D grid cells
- Movement: 4-connected (up/down/left/right)
- Heuristic: Manhattan distance
- Output: ordered list of grid cells from start to goal

## Dynamic replanning (Phase 2)

- Periodically re-run A* from agent's current grid cell
- Temporarily block predicted obstacle cells
- Swap agent's waypoint list with the new plan

## DWA / VO (Phase 3)

- Global planner gives coarse direction
- Local planner samples feasible velocities
- Simulates short-horizon trajectories
- Scores candidates by:
  - distance to goal
  - clearance from obstacles
  - forward progress
