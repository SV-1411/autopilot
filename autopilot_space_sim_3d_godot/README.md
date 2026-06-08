# Autopilot Space Sim 3D

A 3D testbed for an **autonomous spacecraft autopilot** flying through a field of
**moving asteroids**. You set a start and goal, scatter asteroids (with their own
velocities, sizes, masses), press **Go**, and watch the ship plan and fly a safe,
fast route — continuously re-planning as the rocks move.

Built in **Godot 4.5**. Open `project.godot` in Godot and press Play (main scene
is `scenes/Main.tscn`).

---

## The autopilot (how it works)

The guidance system has two layers, exactly like real robot/vehicle autopilots:

1. **Global planner — Voxel A\*** (`scripts/planning/VoxelAStar.gd`)
   Searches a 3D voxel grid for a coarse, collision-free *corridor* from the
   ship to the goal. It runs against a **predicted occupancy grid**: each
   asteroid's future positions (sampled over the next ~2 s) are stamped into the
   grid, so the corridor already leans away from where the rocks are heading.
   Re-runs every `REPLAN_INTERVAL_S` (1 s).

2. **Local planner — 3D Dynamic Window** (`scripts/planning/LocalPlanner3D.gd`)
   The reactive "brain". Every physics tick it:
   - samples thrust commands the ship can physically apply (limited by
     `max_thrust / mass` and top speed),
   - rolls each candidate forward over a short horizon **while predicting where
     every asteroid will be** (`scripts/planning/Predictor.gd`),
   - rejects any predicted collision and scores the rest on
     **progress + clearance + speed + fuel (Δv)**,
   - applies the lowest-cost thrust.

   This is what makes the ship *dodge moving asteroids* rather than just follow a
   line. It is fully classical and explainable — no trained model required.

The **ship** (`scripts/Ship.gd`) is a thrust-limited point mass: it has inertia
and cannot stop or turn instantly, which is what makes this a real guidance
problem. The **asteroids** (`scripts/Asteroid.gd`) move at constant velocity and
bounce off the world bounds; their motion is kept exactly consistent with the
`Predictor` so look-ahead is accurate.

---

## Controls

| Input | Action |
|---|---|
| **Go / Space** | Toggle EDIT ↔ RUN |
| **LMB** | Select ship / asteroid |
| **Shift + LMB** | Set goal at cursor |
| **Ctrl + LMB** | Set start at cursor |
| **A** | Add one asteroid at cursor (edit its velocity in the Inspector) |
| **B** | Generate a random moving asteroid belt |
| **C** | Clear all asteroids |
| **Alt + drag** | Move selected asteroid in 3D |
| **R / F** | Nudge selected up / down |
| **Q / E** | Decrease / increase placement depth (for empty space) |
| **RMB drag** | Orbit camera · **MMB drag** pan · **wheel** zoom |

The **Inspector** edits the selected object's position, velocity, mass, and
radius; **Apply to Selected** commits and re-plans. **Save / Load Scenario**
buttons persist the whole setup (start, goal, asteroids, ship params) to JSON.

## Metrics (shown live during a run)

Status (FLYING / ARRIVED / COLLISION), elapsed time, current speed, remaining
path length, **minimum clearance** to any asteroid, collision count, **Δv used**
(a fuel proxy), asteroid count, and replan count.

---

## Tuning the autopilot

All planner weights live in `LocalPlanner3D.DEFAULT_CFG` and can be overridden
per-ship via the `planner_cfg` export on the Ship node:

| Knob | Effect |
|---|---|
| `safe_margin` | Clearance (m) the ship tries to keep around its hull |
| `w_clear` | How strongly it avoids crowding asteroids (higher = more cautious) |
| `w_goal` | Pull toward the target |
| `w_speed` | Reward for moving fast toward the goal |
| `w_effort` | Penalty on thrust (higher = more fuel-efficient, lazier) |
| `horizon` / `sim_dt` | How far / finely it looks ahead when scoring |
| `n_dirs` | Number of thrust directions sampled (higher = smoother but slower) |

Ship limits (`mass_kg`, `max_thrust_n`, `max_speed_mps`, `target_speed_mps`,
`ship_radius_m`) are exports on the Ship node.

---

## Architecture (files)

The simulation is split into a **headless engine** (no scene/UI) and a **view**.
This is what lets the interactive scene and the benchmark run *identical* logic.

```
scripts/
  SimWorld.gd             HEADLESS ENGINE: owns ship+asteroid state, runs the
                          closed loop (asteroid motion, A* replan, local planner,
                          collision + metrics). Also scenario save/load + belt gen.
  Main.gd                 VIEW/EDITOR: builds a SimWorld from the scene, steps it,
                          syncs node visuals, handles editing/selection/save/load.
  Ship.gd                 ship visual + design params (no logic)
  Asteroid.gd             asteroid visual + design params (no logic)
  OrbitCameraRig.gd       camera
  planning/
    VoxelAStar.gd         global planner (3D grid A*)
    LocalPlanner3D.gd     reactive moving-obstacle avoidance (3D dynamic window)
    Predictor.gd          obstacle trajectory prediction (shared by both planners)
tools/
  BatchEval.gd            headless benchmark over N random scenarios
```

## Benchmarking (headless)

Run many randomized belts and report aggregate autopilot performance — use this
after any change to check for regressions, or to compare planners later.

```
godot --headless --script res://tools/BatchEval.gd -- --runs 200 --asteroids 18 --seed 0
godot --headless --script res://tools/BatchEval.gd -- --runs 100 --out res://eval_results.csv
```

Reports success / collision / timeout rates, min-clearance stats (worst / mean /
p10), Δv (fuel) and time-to-goal. Deterministic per `--seed`. Example (30 belts
of 18 moving asteroids): **96.7% success, 0 collisions, worst clearance 5.8 m**.

## Roadmap / ideas

- [x] Scenario save/load (JSON) for repeatable benchmarks.
- [x] Batch evaluation: run N random belts headless, report success / clearance / Δv.
- Reduce NO_PATH cases: the global A* blocks predicted occupancy over several
  future times, which can over-saturate dense belts. Inflate less / weight
  near-term predictions.
- Swept-volume collision in the global grid (currently samples discrete times).
- Velocity-Obstacle / ORCA local planner as an alternative to the dynamic window.
- Asteroid–asteroid collisions and rotation.
- Optional learned controller (RL) trained in Python, compared against the
  classical planner on the same scenarios (BatchEval is the shared yardstick).
