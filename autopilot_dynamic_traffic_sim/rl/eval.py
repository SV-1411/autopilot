"""Evaluate a trained policy and (optionally) store run data for inspection.

Reports success / collision / timeout rates plus reward and episode length so
you can judge accuracy and see *where* the agent still fails. With
``--save-trajectories`` it dumps every step (agent pose + obstacle states) to
CSV, which you can replay in the 2D/3D viewers or analyze offline.

Usage:
    python -m rl.eval --model runs/models/best.zip --episodes 100
    python -m rl.eval --model runs/models/best.zip --save-trajectories
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

try:
    from stable_baselines3 import PPO
except ModuleNotFoundError as exc:  # pragma: no cover
    raise ModuleNotFoundError(
        "stable-baselines3 is required for eval. Install with "
        "`pip install -r rl/requirements-train.txt`."
    ) from exc

from rl.env import AutopilotGymEnv

TRAJ_DIR = Path("runs") / "eval_trajectories"


def _dump_trajectory(path: Path, rows: list[list]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["step", "ax", "ay", "heading", "speed", "gx", "gy",
                    "obstacles_x_y_vx_vy..."])
        w.writerows(rows)


def evaluate(model_path: str, episodes: int, seed: int, save_traj: bool) -> dict:
    model = PPO.load(model_path)
    env = AutopilotGymEnv()

    goal = collision = timeout = 0
    rewards, lengths = [], []

    for ep in range(episodes):
        obs, _ = env.reset(seed=seed + ep)
        done = False
        ep_r, ep_l = 0.0, 0
        info: dict = {}
        rows: list[list] = []

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            ep_r += float(reward)
            ep_l += 1
            done = terminated or truncated

            if save_traj:
                flat_obs = []
                for o in env._obstacles:  # noqa: SLF001 - intentional for logging
                    flat_obs += [round(o.x, 2), round(o.y, 2),
                                 round(o.vx, 2), round(o.vy, 2)]
                rows.append([ep_l, round(env._ax, 2), round(env._ay, 2),
                             round(env._heading, 3), round(env._speed, 2),
                             round(env._gx, 2), round(env._gy, 2), *flat_obs])

        reason = info.get("terminal_reason")
        goal += reason == "goal"
        collision += reason == "collision"
        timeout += reason == "timeout"
        rewards.append(ep_r)
        lengths.append(ep_l)

        if save_traj:
            _dump_trajectory(TRAJ_DIR / f"episode_{ep:03d}_{reason}.csv", rows)

    n = episodes
    summary = {
        "episodes": n,
        "success_rate": goal / n,
        "collision_rate": collision / n,
        "timeout_rate": timeout / n,
        "mean_reward": float(np.mean(rewards)),
        "mean_episode_len": float(np.mean(lengths)),
    }
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate a trained policy.")
    parser.add_argument("--model", type=str, default="runs/models/best.zip")
    parser.add_argument("--episodes", type=int, default=100)
    parser.add_argument("--seed", type=int, default=10_000)
    parser.add_argument("--save-trajectories", action="store_true",
                        help="Write per-step CSV for each episode.")
    args = parser.parse_args()

    s = evaluate(args.model, args.episodes, args.seed, args.save_trajectories)

    print(f"\nEvaluated {s['episodes']} episodes of {args.model}")
    print(f"  success   : {s['success_rate']:.1%}")
    print(f"  collision : {s['collision_rate']:.1%}")
    print(f"  timeout   : {s['timeout_rate']:.1%}")
    print(f"  mean reward     : {s['mean_reward']:.2f}")
    print(f"  mean episode len: {s['mean_episode_len']:.1f}")
    if args.save_trajectories:
        print(f"  trajectories saved -> {TRAJ_DIR}")


if __name__ == "__main__":
    main()
