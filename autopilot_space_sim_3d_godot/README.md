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

1. **Global planner — time-indexed A\*** (`scripts/planning/SpaceTimeAStar.gd`)
   Searches a 3D voxel grid for a coarse, collision-free *corridor* from the
   ship to the goal — but checks each cell's occupancy **at the time the ship
   would arrive there**, not "ever". Predicted occupancy is built as a stack of
   time layers (one per 0.2 s, sphere-stamped, optionally inflated by a k·σ
   uncertainty shell); a node reached at path cost g maps to the layer at
   t = g · (voxel / cruise speed). This threads gaps that close later and
   crosses cells that clear before arrival — the legacy "union" model (kept as
   a benchmark canary, `scripts/planning/VoxelAStar.gd`) falsely refused ~7% of
   dense belts. All-integer packed-heap search, hard per-replan time budget
   (`plan_deadline_usec`); on failure the previous path is validated against
   fresh predictions and truncated, or guidance degrades to local-only mode
   (surfaced via the `degraded` flag). Re-runs every `replan_interval` (1 s).

2. **Local planner — 3D Dynamic Window** (`scripts/planning/LocalPlanner3D.gd`)
   The reactive "brain". Every physics tick it:
   - samples thrust commands the ship can physically apply (limited by
     `max_thrust / mass` and top speed),
   - rolls each candidate forward over a short horizon **while predicting where
     every asteroid will be** (`scripts/planning/Predictor.gd`), culling
     obstacles that cannot physically interact within the horizon,
   - rejects any predicted collision using **swept checks** (closest approach of
     the relative-motion segment — tunneling through a rock between samples is
     impossible at any speed) and scores the rest on
     **progress + clearance + speed + fuel (Δv)**,
   - applies the lowest-cost thrust; if *every* candidate is doomed, it flies
     the **least-bad maneuver** (greatest closest-approach) rather than blindly
     braking into a pursuer.

   **Chance-constrained mode** (`unc_enable`): each obstacle's effective radius
   grows by `k·(σ₀ + growth·t)` over the look-ahead, keeping the ship outside
   the k-sigma uncertainty shell — the same principle operational conjunction
   avoidance uses. Pair with process noise (`noise_sigma`, applied √dt-scaled to
   asteroid velocities) to simulate imperfect tracking.

   Ground truth uses the same swept collision math (`SimWorld._update_metrics`),
   and the asteroid integrator delegates to `Predictor.advance` so simulated
   truth and the planners' predictions share one reflection law by construction
   (pinned by the `predictor_equals_integrator` self-test).

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
                          closed loop (asteroid motion, replan, local planner,
                          swept collision + metrics, stale-path guard, planner
                          telemetry). Also scenario save/load (schema v2) + belt gen.
  Main.gd                 VIEW/EDITOR: builds a SimWorld from the scene, steps it,
                          syncs node visuals, draws the corridor + predicted
                          asteroid ghost trails, realism controls, telemetry HUD.
  Ship.gd                 ship visual + design params (no logic)
  Asteroid.gd             asteroid visual + design params (no logic)
  OrbitCameraRig.gd       camera
  planning/
    SpaceTimeAStar.gd     global planner (time-indexed weighted A*, packed-heap)
    VoxelAStar.gd         legacy union-grid A* (benchmark canary)
    LocalPlanner3D.gd     reactive moving-obstacle avoidance (3D dynamic window,
                          swept checks, chance constraints, least-bad fallback)
    Predictor.gd          obstacle trajectory prediction AND integration
                          (one reflection law for truth + prediction)
tools/
  run_headless.ps1        fail-fast runner (import gate, boot sentinel, timeout,
                          real exit codes) -- use this for everything below
  SelfTest.gd             invariant test suite (13 deterministic scenarios)
  BatchEval.gd            headless benchmark: gates, failure dumps, replay,
                          noise/uncertainty knobs, planner A/B canary
```

## Testing & benchmarking (headless)

**Always run headless tools through the fail-fast wrapper** — it re-imports the
project first (a `class_name` script added without re-import makes headless runs
hang silently), requires a boot sentinel, applies a hard timeout, and propagates
real exit codes:

```powershell
# Invariant test suite (13 deterministic scenarios; exit 1 on any failure):
& tools\run_headless.ps1 -Script res://tools/SelfTest.gd

# Benchmark: many randomized belts, aggregate stats:
& tools\run_headless.ps1 -Script res://tools/BatchEval.gd `
    -ToolArgs "--runs 60 --asteroids 26 --seed 0"

# Regression-gated run (nonzero exit on violation), with failure dumps:
& tools\run_headless.ps1 -Script res://tools/BatchEval.gd `
    -ToolArgs "--runs 60 --asteroids 26 --seed 0 --gate-success 95 --gate-collisions 0 --gate-nopath 2 --dump-failures res://eval_failures"

# Replay one dumped failure with full detail (exit 0 iff it now arrives):
& tools\run_headless.ps1 -Script res://tools/BatchEval.gd `
    -ToolArgs "--replay res://eval_failures/fail_007_COLLISION.json"

# Realism: process noise + chance-constrained avoidance:
& tools\run_headless.ps1 -Script res://tools/BatchEval.gd `
    -ToolArgs "--runs 60 --asteroids 26 --noise 0.5 --uncertainty"

# Legacy-planner canary (must reproduce the old ~7% false NO_PATH rate):
& tools\run_headless.ps1 -Script res://tools/BatchEval.gd `
    -ToolArgs "--runs 60 --asteroids 26 --planner union"
```

Reports success / collision / timeout / NO_PATH rates, min-clearance stats
(worst / mean / p10, measured with swept closest-approach), Δv (fuel),
time-to-goal, and planner telemetry (replan ms, failures, degraded runs).
Deterministic per `--seed`. Scenario JSON (schema v2) carries bounds, noise,
seed, and uncertainty config so dumps replay faithfully.

Reference result (60 belts of 26 moving asteroids, seed 0, time-indexed
planner): **100% success, 0 collisions, 0 false refusals** — the legacy union
planner scores 93.3% with 6.7% NO_PATH on the identical belts.

### Verification matrix sign-off (2026-06-11, `tools/run_matrix.ps1`, 0 failures)

| Cell | Config | Success | Coll. | NO_PATH | Worst clear | Replan mean |
|---|---|---|---|---|---|---|
| union canary | legacy planner, 26 ast. | 96.7% | 0 | 3.3% | 0.22 m | 60.5 ms |
| dense stress | time planner, 34 ast. | 100% | 0 | 0% | 2.59 m | 11.4 ms |
| noise, det. margins | σ=0.5 m/s² drift | 100% | 0 | 0% | 0.78 m | 8.1 ms |
| noise + 3σ shells | σ=0.5, chance-constr. | 98.3% | 0 | 1.7% | 1.03 m (mean 8.60) | 26.4 ms |
| sparse ×100 | 18 ast., 100 runs | 99% | 0 | 1% | 2.18 m | 6.2 ms |

vs. the original pre-rewrite 100-run baseline (92% success / 1 collision /
7% NO_PATH): success +7 pts, collisions eliminated, refusals 7× fewer.
Cells 3 vs 4 isolate the chance-constraint payoff on identical noisy belts:
mean clearance 6.49 → 8.60 m for ~6% extra Δv. Zero degraded runs in every
time-planner cell (280 flights); the union canary logged 1.

## Roadmap / ideas

- [x] Scenario save/load (JSON, schema v2) for repeatable benchmarks.
- [x] Batch evaluation: run N random belts headless, report success / clearance / Δv.
- [x] Reduce NO_PATH cases — time-indexed occupancy + sphere stamping eliminated
  the false refusals (6.7% → 0% on the 26-asteroid reference config).
- [x] Swept (tunneling-free) collision in the local planner AND ground truth.
- [x] Uncertainty: process noise + chance-constrained (k·σ) avoidance.
- [x] Invariant self-test suite + fail-fast headless toolchain + benchmark gates.
- Inter-layer sweep stamping in the global grid (layers sample instants; the
  swept local planner is the safety net in between).
- Velocity-Obstacle / ORCA local planner as an alternative to the dynamic window.
- Asteroid–asteroid collisions and rotation.
- SI/Keplerian dynamics, sensor-limited perception (range/FOV + tracking filter),
  Monte-Carlo collision-probability reporting — the path to ops-grade realism.
- Optional learned controller (RL) trained in Python, compared against the
  classical planner on the same scenarios (BatchEval is the shared yardstick).
