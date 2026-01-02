from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim

from rl.env import AutopilotGymEnv


@dataclass
class Transition:
    s: np.ndarray
    a: int
    r: float
    ns: np.ndarray
    d: bool


class ReplayBuffer:
    def __init__(self, capacity: int = 100_000) -> None:
        self.capacity = int(capacity)
        self.data: List[Transition] = []
        self.i = 0

    def push(self, t: Transition) -> None:
        if len(self.data) < self.capacity:
            self.data.append(t)
        else:
            self.data[self.i] = t
        self.i = (self.i + 1) % self.capacity

    def sample(self, batch_size: int) -> List[Transition]:
        return random.sample(self.data, batch_size)

    def __len__(self) -> int:
        return len(self.data)


class QNet(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int) -> None:
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ReLU(),
            nn.Linear(256, 256),
            nn.ReLU(),
            nn.Linear(256, act_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


def train() -> None:
    env = AutopilotGymEnv()

    obs_dim = env.obs_dim
    act_dim = env.action_dim

    q = QNet(obs_dim, act_dim)
    tq = QNet(obs_dim, act_dim)
    tq.load_state_dict(q.state_dict())

    opt = optim.Adam(q.parameters(), lr=3e-4)
    rb = ReplayBuffer(100_000)

    gamma = 0.99
    batch_size = 128
    target_update = 500

    eps_start = 1.0
    eps_end = 0.05
    eps_decay_steps = 50_000

    total_steps = 60_000

    s = env.reset()
    for step in range(1, total_steps + 1):
        eps = eps_end + (eps_start - eps_end) * math.exp(-step / eps_decay_steps)
        if random.random() < eps:
            a = random.randrange(act_dim)
        else:
            with torch.no_grad():
                qs = q(torch.from_numpy(s).float().unsqueeze(0))
                a = int(torch.argmax(qs, dim=1).item())

        ns, r, d, _ = env.step(a)
        rb.push(Transition(s=s, a=a, r=r, ns=ns, d=d))
        s = ns if not d else env.reset()

        if len(rb) >= batch_size:
            batch = rb.sample(batch_size)

            sb = torch.from_numpy(np.stack([t.s for t in batch])).float()
            ab = torch.tensor([t.a for t in batch], dtype=torch.int64)
            rbw = torch.tensor([t.r for t in batch], dtype=torch.float32)
            nsb = torch.from_numpy(np.stack([t.ns for t in batch])).float()
            db = torch.tensor([t.d for t in batch], dtype=torch.float32)

            qv = q(sb).gather(1, ab.view(-1, 1)).squeeze(1)

            with torch.no_grad():
                next_q = tq(nsb).max(dim=1).values
                target = rbw + gamma * (1.0 - db) * next_q

            loss = nn.functional.smooth_l1_loss(qv, target)

            opt.zero_grad()
            loss.backward()
            nn.utils.clip_grad_norm_(q.parameters(), 5.0)
            opt.step()

        if step % target_update == 0:
            tq.load_state_dict(q.state_dict())

        if step % 2000 == 0:
            print(f"step={step} eps={eps:.3f} buffer={len(rb)}")


if __name__ == "__main__":
    train()
