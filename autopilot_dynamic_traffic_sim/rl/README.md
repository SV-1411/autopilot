# RL / Phase 4

## What this is

- `env.py`: a headless (no Pygame) environment with `reset()` and `step(action)` suitable for running in Google Colab.
- `colab_dqn.py`: a minimal DQN training template using PyTorch (CPU). Intended to run in Colab.

## Quick usage (Colab)

1. Upload this repo to GitHub (or zip and upload to Colab)
2. In Colab:

```bash
pip install torch numpy
```

3. Run:

```bash
python -m rl.colab_dqn
```

Notes:

- This is intentionally minimal. You’ll likely want to normalize observations and tune rewards.
- Keep training in Colab; your laptop can run the simulation locally.
