"""Train (or continue training) the autopilot navigation policy with PPO.

Designed so the agent gets *more accurate every run*:

* ``--resume`` loads an existing model and keeps learning from where it left
  off (timesteps and the optimizer/value baseline carry over).
* Every run writes data so you can see improvement and diagnose failures:
    - runs/checkpoints/      periodic model snapshots (.zip)
    - runs/models/           latest + best models (.zip)
    - runs/tb/               TensorBoard logs (full learning curves)
    - runs/monitor/          per-episode reward/length (SB3 Monitor CSVs)
    - runs/metrics_history.csv  cumulative eval metrics across ALL runs

Usage:
    python -m rl.train                                  # fresh run, 1M steps
    python -m rl.train --timesteps 2000000
    python -m rl.train --resume runs/models/latest.zip  # continue improving
    python -m rl.train --n-envs 8 --eval-freq 50000

Watch progress live:
    tensorboard --logdir runs/tb
"""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import numpy as np

try:
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
    from stable_baselines3.common.env_util import make_vec_env
except ModuleNotFoundError as exc:  # pragma: no cover
    raise ModuleNotFoundError(
        "stable-baselines3 / torch are required for training. Install with "
        "`pip install -r rl/requirements-train.txt`."
    ) from exc

from rl.env import AutopilotGymEnv

RUNS_DIR = Path("runs")
MODELS_DIR = RUNS_DIR / "models"
CKPT_DIR = RUNS_DIR / "checkpoints"
TB_DIR = RUNS_DIR / "tb"
MONITOR_DIR = RUNS_DIR / "monitor"
METRICS_CSV = RUNS_DIR / "metrics_history.csv"

LATEST_MODEL = MODELS_DIR / "latest.zip"
BEST_MODEL = MODELS_DIR / "best.zip"


class EvalAndLogCallback(BaseCallback):
    """Periodically evaluate the greedy policy, log metrics, and save the best.

    The metrics CSV is *appended to* across runs, so the file is a running
    history of how the agent improves. The per-episode terminal-reason
    breakdown (goal / collision / timeout) tells you *where* to improve:
        high collision_rate  -> avoidance is weak (more obstacle shaping)
        high timeout_rate    -> too timid/slow (more progress reward)
    """

    def __init__(
        self,
        eval_env: AutopilotGymEnv,
        run_id: str,
        eval_freq: int,
        n_eval_episodes: int = 30,
        curriculum: list[tuple[float, float]] | None = None,
        advance_threshold: float = 0.7,
        verbose: int = 1,
    ) -> None:
        super().__init__(verbose)
        self.eval_env = eval_env
        self.run_id = run_id
        self.eval_freq = max(1, int(eval_freq))
        self.n_eval_episodes = int(n_eval_episodes)
        self._best_score = -float("inf")

        # Curriculum: list of (speed_min, speed_max) stages, easiest first.
        # Advance to the next stage once success crosses advance_threshold.
        self.curriculum = curriculum
        self.advance_threshold = float(advance_threshold)
        self._stage = 0

    def _on_step(self) -> bool:
        if self.n_calls % self.eval_freq != 0:
            return True

        metrics = self._evaluate()
        self._record(metrics)
        self._maybe_save_best(metrics)
        self._maybe_advance_curriculum(metrics)
        return True

    def _maybe_advance_curriculum(self, m: dict) -> None:
        if not self.curriculum:
            return
        if self._stage >= len(self.curriculum) - 1:
            return
        if m["success_rate"] < self.advance_threshold:
            return

        self._stage += 1
        smin, smax = self.curriculum[self._stage]
        # Apply to every training env and to the eval env.
        self.model.get_env().env_method(
            "set_difficulty", obstacle_speed_min=smin, obstacle_speed_max=smax
        )
        self.eval_env.set_difficulty(obstacle_speed_min=smin, obstacle_speed_max=smax)
        # Reset best tracking so best.zip reflects the new (harder) stage.
        self._best_score = -float("inf")
        if self.verbose:
            print(
                f"  -> CURRICULUM advance to stage {self._stage} "
                f"(obstacle speed {smin:.0f}-{smax:.0f})"
            )

    def _evaluate(self) -> dict:
        rewards, lengths = [], []
        goal = collision = timeout = 0

        for ep in range(self.n_eval_episodes):
            obs, _ = self.eval_env.reset(seed=10_000 + ep)  # fixed eval set
            done = False
            ep_r, ep_l = 0.0, 0
            info: dict = {}
            while not done:
                action, _ = self.model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = self.eval_env.step(action)
                ep_r += float(reward)
                ep_l += 1
                done = terminated or truncated
            reason = info.get("terminal_reason")
            goal += reason == "goal"
            collision += reason == "collision"
            timeout += reason == "timeout"
            rewards.append(ep_r)
            lengths.append(ep_l)

        n = self.n_eval_episodes
        return {
            "run_id": self.run_id,
            "timesteps": int(self.num_timesteps),
            "mean_reward": float(np.mean(rewards)),
            "success_rate": goal / n,
            "collision_rate": collision / n,
            "timeout_rate": timeout / n,
            "mean_episode_len": float(np.mean(lengths)),
        }

    def _record(self, m: dict) -> None:
        # TensorBoard
        self.logger.record("eval/mean_reward", m["mean_reward"])
        self.logger.record("eval/success_rate", m["success_rate"])
        self.logger.record("eval/collision_rate", m["collision_rate"])
        self.logger.record("eval/timeout_rate", m["timeout_rate"])

        # Cumulative CSV history (append; create header once)
        METRICS_CSV.parent.mkdir(parents=True, exist_ok=True)
        new_file = not METRICS_CSV.exists()
        with METRICS_CSV.open("a", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(
                    ["wall_time", "run_id", "timesteps", "mean_reward",
                     "success_rate", "collision_rate", "timeout_rate",
                     "mean_episode_len", "stage", "obstacle_speed_max"]
                )
            w.writerow([
                round(time.time(), 1), m["run_id"], m["timesteps"],
                round(m["mean_reward"], 3), round(m["success_rate"], 4),
                round(m["collision_rate"], 4), round(m["timeout_rate"], 4),
                round(m["mean_episode_len"], 1),
                self._stage, round(self.eval_env.obstacle_speed_max, 1),
            ])

        if self.verbose:
            print(
                f"[eval @ {m['timesteps']:>9} steps] "
                f"reward={m['mean_reward']:7.2f}  "
                f"success={m['success_rate']:.0%}  "
                f"collision={m['collision_rate']:.0%}  "
                f"timeout={m['timeout_rate']:.0%}"
            )

    def _maybe_save_best(self, m: dict) -> None:
        # Rank by success rate, break ties with mean reward.
        score = m["success_rate"] * 1000.0 + m["mean_reward"]
        if score > self._best_score:
            self._best_score = score
            MODELS_DIR.mkdir(parents=True, exist_ok=True)
            self.model.save(BEST_MODEL)
            if self.verbose:
                print(f"  -> new best (success={m['success_rate']:.0%}); saved {BEST_MODEL}")


def make_envs(n_envs: int, seed: int, env_kwargs: dict):
    MONITOR_DIR.mkdir(parents=True, exist_ok=True)
    train_env = make_vec_env(
        AutopilotGymEnv,
        n_envs=n_envs,
        seed=seed,
        monitor_dir=str(MONITOR_DIR),
        env_kwargs=env_kwargs,
    )
    eval_env = AutopilotGymEnv(**env_kwargs)
    return train_env, eval_env


def build_model(
    train_env, resume: str | None, seed: int, ent_coef: float,
    log_std_init: float, net_arch: list[int],
) -> PPO:
    if resume:
        print(f"Resuming from {resume}")
        model = PPO.load(resume, env=train_env, tensorboard_log=str(TB_DIR))
        model.ent_coef = ent_coef  # allow raising exploration on resume
        return model
    return PPO(
        "MlpPolicy",
        train_env,
        seed=seed,
        verbose=1,
        n_steps=2048,
        batch_size=256,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        ent_coef=ent_coef,
        learning_rate=3e-4,
        policy_kwargs=dict(net_arch=list(net_arch), log_std_init=log_std_init),
        tensorboard_log=str(TB_DIR),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Train the autopilot PPO policy.")
    parser.add_argument("--timesteps", type=int, default=1_000_000)
    parser.add_argument("--n-envs", type=int, default=8)
    parser.add_argument("--eval-freq", type=int, default=50_000,
                        help="Evaluate every N total timesteps.")
    parser.add_argument("--n-eval-episodes", type=int, default=30)
    parser.add_argument("--resume", type=str, default=None,
                        help="Path to a .zip model to continue training.")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--run-name", type=str, default=None)
    parser.add_argument("--no-progress", action="store_true",
                        help="Disable the live progress bar (use for logs).")
    # Exploration
    parser.add_argument("--ent-coef", type=float, default=0.01,
                        help="PPO entropy bonus; higher = more exploration.")
    parser.add_argument("--log-std-init", type=float, default=-1.0,
                        help="Initial action log-std. Lower = tighter, more "
                             "deliberate actions (SB3 default is 0.0 -> std 1.0).")
    parser.add_argument("--net-arch", type=int, nargs="+", default=[256, 256],
                        help="Hidden layer sizes, e.g. --net-arch 512 512.")
    # Environment difficulty (obstacle count is fixed; it sets observation size)
    parser.add_argument("--n-obstacles", type=int, default=8)
    parser.add_argument("--obstacle-speed-min", type=float, default=20.0)
    parser.add_argument("--obstacle-speed-max", type=float, default=60.0)
    parser.add_argument("--goal-radius", type=float, default=16.0)
    # Reward shaping: higher progress-gain pulls harder toward the goal;
    # higher time-penalty punishes stalling/timeouts.
    parser.add_argument("--progress-gain", type=float, default=0.05)
    parser.add_argument("--time-penalty", type=float, default=0.01)
    # Dense obstacle-avoidance shaping (0 disables it).
    parser.add_argument("--clearance-gain", type=float, default=0.0)
    parser.add_argument("--safe-margin", type=float, default=30.0)
    # Curriculum: ramp obstacle speed up as the agent succeeds. Each stage is
    # "min:max"; training starts on stage 0 and advances when eval success
    # crosses --advance-threshold.
    parser.add_argument("--curriculum", type=str, default=None,
                        help='Speed stages, e.g. "0:10,10:30,20:60". '
                             "Omit for fixed difficulty.")
    parser.add_argument("--advance-threshold", type=float, default=0.7)
    args = parser.parse_args()

    for d in (MODELS_DIR, CKPT_DIR, TB_DIR, MONITOR_DIR):
        d.mkdir(parents=True, exist_ok=True)

    run_id = args.run_name or time.strftime("run_%Y%m%d_%H%M%S")

    # Parse the curriculum ("min:max,min:max,...") if provided.
    curriculum: list[tuple[float, float]] | None = None
    if args.curriculum:
        curriculum = []
        for stage in args.curriculum.split(","):
            lo, hi = stage.split(":")
            curriculum.append((float(lo), float(hi)))

    env_kwargs = dict(
        n_obstacles=args.n_obstacles,
        obstacle_speed_min=args.obstacle_speed_min,
        obstacle_speed_max=args.obstacle_speed_max,
        goal_radius=args.goal_radius,
        progress_gain=args.progress_gain,
        time_penalty=args.time_penalty,
        clearance_gain=args.clearance_gain,
        safe_margin=args.safe_margin,
    )
    # Start on the easiest curriculum stage if one was given.
    if curriculum:
        env_kwargs["obstacle_speed_min"] = curriculum[0][0]
        env_kwargs["obstacle_speed_max"] = curriculum[0][1]

    train_env, eval_env = make_envs(args.n_envs, args.seed, env_kwargs)
    model = build_model(
        train_env, args.resume, args.seed, args.ent_coef, args.log_std_init,
        args.net_arch,
    )

    # eval_freq is per-rollout-step (shared across envs), so divide by n_envs.
    eval_freq_steps = max(1, args.eval_freq // args.n_envs)

    callbacks = [
        CheckpointCallback(
            save_freq=max(1, (args.timesteps // 10) // args.n_envs),
            save_path=str(CKPT_DIR),
            name_prefix="ckpt",
        ),
        EvalAndLogCallback(
            eval_env=eval_env,
            run_id=run_id,
            eval_freq=eval_freq_steps,
            n_eval_episodes=args.n_eval_episodes,
            curriculum=curriculum,
            advance_threshold=args.advance_threshold,
        ),
    ]

    model.learn(
        total_timesteps=args.timesteps,
        callback=callbacks,
        reset_num_timesteps=args.resume is None,  # keep counter when resuming
        tb_log_name=run_id,
        progress_bar=not args.no_progress,
    )

    model.save(LATEST_MODEL)
    print(f"\nSaved latest model -> {LATEST_MODEL}")
    print(f"Best model         -> {BEST_MODEL}")
    print(f"Metrics history    -> {METRICS_CSV}")
    print("Continue improving later with:")
    print(f"  python -m rl.train --resume {LATEST_MODEL}")


if __name__ == "__main__":
    main()
