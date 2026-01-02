# Usage

## Run

```bash
pip install -r requirements.txt
python main.py
```

## What you’ll see (Phase 3)

- **Green triangle**: your agent (car/ship) with a heading line
- **Yellow line**: DWA local trajectory (predicted path over ~1.5s)
- **Blue line**: global A* path from start to goal
- **Red circles**: moving obstacles
- **Yellow dot**: start; **Purple dot**: goal

## How the autopilot works

- **Global planner**: A* gives a waypoint path from start to goal
- **Local planner**: DWA chooses velocities to avoid moving obstacles while heading toward the next waypoint
- **Dynamic replanning**: Every `REPLAN_INTERVAL` seconds, the system predicts where moving obstacles will be and re-runs A* from the agent’s current cell
- **Collision handling**: If the agent hits an obstacle or a wall, it resets to the start

## Configuration knobs (`config.py`)

- `USE_DWA = True/False`: toggle DWA local planner (False = pure waypoint following)
- `MOVING_OBSTACLE_COUNT`: number of moving obstacles
- `MOVING_OBSTACLE_MIN_SPEED` / `MOVING_OBSTACLE_MAX_SPEED`: obstacle speed range
- `REPLAN_INTERVAL`: how often to re-run global A* (seconds)
- `REPLAN_PREDICT_TIME`: how far ahead to predict obstacles for replanning
- `DWA_CONFIG`: DWA parameters (speeds, resolution, cost gains)
- `STATIC_OBSTACLE_SAMPLE_RADIUS_CELLS`: how far around the agent to sample static grid cells as obstacles for DWA

- `DENSE_STATIC_MODE`: start with a dense random static obstacle map
- `DENSE_STATIC_FILL_PROB`: obstacle density (higher = more blocked)
- `DENSE_MOVING_OBSTACLE_COUNT`: moving obstacle count when dense mode is enabled

- `PATH_SMOOTHING`: post-process A* path to remove zig-zags (line-of-sight shortcutting)
- `PATH_SMOOTHING_MAX_SKIP`: how aggressively to shortcut along the path

## Manual controls (Phase 4)

- **Left click**: add a static grid obstacle at the mouse cell
- **Right click**: remove a static grid obstacle at the mouse cell
- **Shift + Left click**: move the goal to the mouse cell
- **O**: respawn moving obstacles
- **G**: toggle dense random static map (fills the screen with obstacles) and replan
- **R**: regenerate the current map and respawn obstacles
- **[ / ]**: decrease/increase dense obstacle fill probability
- **- / =**: decrease/increase moving obstacle count

Every manual edit forces an immediate replan.

## How to test

1. **Baseline**: Set `USE_DWA = False`. You should see the agent follow the blue A* path directly, ignoring moving obstacles.
2. **DWA enabled**: Set `USE_DWA = True`. The agent should dodge moving obstacles while still progressing toward the goal.
3. **Stress test**: Increase `MOVING_OBSTACLE_COUNT` to 12–16 and/or increase `MOVING_OBSTACLE_MAX_SPEED`. The agent should still reach the goal, possibly with more replans.
4. **Replan rate**: Lower `REPLAN_INTERVAL` to 0.1 for more frequent replanning; raise to 0.5 for less frequent.
5. **Static scenario**: Edit `core/environment.py` -> `_build_static_scenario()` to add/remove walls or change obstacle layout.

## What to observe

- Does the agent reach the goal?
- How often does it replan (you can add a print in `_plan_path` to see)
- Does it avoid collisions?
- Does the yellow DWA trajectory look reasonable (not going through obstacles)?

## Next steps (optional)

- **Continuous mode**: after reaching the goal, pick a new goal so the agent keeps navigating
- **Metrics display**: show collisions, replans, time-to-goal on screen
- **Traffic patterns**: add lanes or swarm behaviors for obstacles
- **RL/Colab**: export a gym-like `reset()/step()` interface for reinforcement learning

## RL / Colab (Phase 4)

This repo includes a headless environment and a minimal DQN training template:

- `rl/env.py`: `AutopilotGymEnv` with `reset()` and `step(action)` (no Pygame)
- `rl/colab_dqn.py`: CPU-only PyTorch DQN training loop

Run locally (if you install torch) or in Google Colab:

```bash
pip install torch numpy
python -m rl.colab_dqn
```

## 3D designs (recommended approach)

Keep your planning + simulation logic in Python (this repo) and do visuals separately.

- **Low-end laptop friendly**
  - Model assets in **Blender** (car/ship, obstacles, environment)
  - Keep them low-poly; export `.fbx` or `.glb`

- **3D simulation / demo**
  - Use **Unity** for 3D rendering and camera work
  - Feed Unity with positions over time:
    - simplest: export a CSV log from Python (`t,x,y,yaw` + obstacle states)
    - Unity reads CSV and plays it back

- **CSV export (built-in)**
  - Set `CSV_EXPORT = True` in `config.py`
  - Run the simulation; `trajectory_log.csv` will be written
  - See `docs/3d_workflow.md` for Unity import script and Blender tips

- **If you want “real robotics” style**
  - ROS2 + Gazebo is powerful but heavy; not recommended on your laptop right now
