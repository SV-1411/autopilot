# RL — training the autopilot policy (Phase 4)

The agent learns to drive from start to goal while avoiding moving obstacles,
using **PPO** (Stable-Baselines3). The RL environment shares the **same
unicycle dynamics** (`core/dynamics.py`) and **same observation builder**
(`rl/obs.py`) as the live simulation, so a trained policy can be dropped back
into `main.py` (Phase 4) without behavioral drift.

## Files

| File | Purpose |
|---|---|
| `env.py` | Gymnasium env (continuous actions, shaped reward, moving obstacles) |
| `obs.py` | Shared, normalized, goal-relative observation builder |
| `train.py` | PPO training — resumable, with checkpoints + metrics logging |
| `eval.py` | Measure success/collision/timeout; optionally dump trajectories |
| `requirements-train.txt` | Training dependencies (install in Colab/GPU box) |
| `colab_dqn.py` | **Deprecated** reference DQN (old API; use `train.py` instead) |

## Install

```bash
pip install -r rl/requirements-train.txt
```

## Train

From the project root (`autopilot_dynamic_traffic_sim/`):

```bash
python -m rl.train                       # fresh run, 1M steps, 8 envs
python -m rl.train --timesteps 2000000
```

### Make it more accurate every run (resume)

Each run can continue the previous model — timesteps, value baseline, and
optimizer state carry over, so accuracy keeps climbing:

```bash
python -m rl.train --resume runs/models/latest.zip
```

## What gets stored (run data)

Everything lands under `runs/` (git-ignored):

| Path | What it holds |
|---|---|
| `runs/models/latest.zip` | most recent model — resume from this |
| `runs/models/best.zip` | best model by eval success rate |
| `runs/checkpoints/` | periodic snapshots during training |
| `runs/tb/` | TensorBoard logs (full learning curves) |
| `runs/monitor/` | per-episode reward/length CSVs (SB3 Monitor) |
| `runs/metrics_history.csv` | **cumulative** eval metrics across *all* runs |
| `runs/eval_trajectories/` | per-step CSVs from `eval.py --save-trajectories` |

Watch live:

```bash
tensorboard --logdir runs/tb
```

## Where the agent can improve itself (diagnosing failures)

`runs/metrics_history.csv` (and TensorBoard) break each evaluation into
terminal reasons. Read them like this:

- **high `collision_rate`** → obstacle avoidance is weak. Increase the
  obstacle term / add a clearance penalty in `env.py`, or train longer.
- **high `timeout_rate`** → agent is too timid/slow. Increase `progress_gain`
  or reduce `time_penalty`.
- **`success_rate` plateaus** → raise difficulty (more obstacles / faster
  obstacles via `n_obstacles`, `obstacle_speed_max`) and `--resume` to push
  generalization.

## Evaluate

```bash
python -m rl.eval --model runs/models/best.zip --episodes 100
python -m rl.eval --model runs/models/best.zip --save-trajectories
```

## Quick start in Google Colab (free GPU)

```python
# 1. Get the code (clone your GitHub repo, or upload a zip)
!git clone https://github.com/<you>/<repo>.git
%cd <repo>/autopilot_dynamic_traffic_sim

# 2. Install
!pip install -r rl/requirements-train.txt

# 3. Train (GPU is auto-detected)
!python -m rl.train --timesteps 2000000

# 4. Download the model to deploy locally
from google.colab import files
files.download("runs/models/best.zip")
```

To keep improving across Colab sessions, save `runs/` to Google Drive and pass
`--resume runs/models/latest.zip` next time.

## Deploy (Phase 4)

Copy a trained `.zip` to your machine and set in `config.py`:

```python
CONTROLLER = "rl"
RL_MODEL_PATH = "runs/models/best.zip"
```

Then `python main.py` runs the learned policy in the live sim. (Phase 4 wires
the `RLController` into the agent.)
