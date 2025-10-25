import time
import random
from dataclasses import dataclass
from typing import Tuple, Optional

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from src.utils.config import DEVICE

def set_seed(seed: int = 42):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


class CoppeliaSimObstacleAvoidEnv:
    """
    Ambiente simples de desvio de obstáculos com base em sensores de proximidade
    (ex.: 8 IRs ou varredura LiDAR reduzida). A ação é discreta: (v, ω) escolhidos
    entre combinações pré-definidas para um drive diferencial.
    """

    def __init__(
        self,
        max_steps: int = 300,
        sensor_dim: int = 16,
        v_lin: float = 0.12,
        w_ang: float = 1.2,
        action_scheme: str = "5"
    ):
        self.max_steps = max_steps
        self.sensor_dim = sensor_dim
        self.step_count = 0
        self.v_lin = v_lin
        self.w_ang = w_ang

        if action_scheme == "3":
            self.actions = [
                (1.0,  0.0),
                (0.6, +1.0),
                (0.6, -1.0),
            ]
        else:
            self.actions = [
                (1.0,  0.0),
                (0.8, +0.6),
                (0.8, -0.6),
                (0.4, +1.0),
                (0.4, -1.0),
            ]
        self.n_actions = len(self.actions)
        self.obs_low = np.zeros(self.sensor_dim, dtype=np.float32)
        self.obs_high = np.ones(self.sensor_dim, dtype=np.float32)

        # (TODO: trocar pelo setup) ---
        self.session = None
        self.robot = None
        self.left_joint = None
        self.right_joint = None
        self.body_handle = None
        self.last_position = None
        self.collision = False

    def _connect(self):
        """Conecte à sim e resolva handles (somente uma vez por processo)."""
        if self.session is not None:
            return
        # TODO: Trocar pelo client

    def _reset_sim(self):
        """Reseta a simulação para um estado inicial válido."""
        # TODO: Trocar pelo client
        time.sleep(0.05)

    def _read_sensors(self) -> np.ndarray:
        """Leia sensores e normalize em [0,1] (0 longe, 1 bem perto)."""
        # TODO: ler IR/LiDAR, normalizar/recortar para sensor_dim
        
        obs = np.clip(np.random.rand(self.sensor_dim) * 0.2, 0, 1).astype(np.float32)
        return obs

    def _collided(self) -> bool:
        """Retorne True se colidiu (via sensor/flag do CoppeliaSim)."""
        # TODO: checar sensor de colisão ou distância < epsilon
        return self.collision

    def _set_wheel_speeds(self, v: float, w: float):
        """Converte (v,w) para velocidades de rodas e aplica nos joints."""
        # drive diferencial: v_left = (2v - wL) / (2R), v_right = (2v + wL) / (2R)
        # Use seu raio de roda (R) e distância entre rodas (L).
        R = 0.0205
        L = 0.053 
        v_l = (2.0 * v - w * L) / (2.0 * R)
        v_r = (2.0 * v + w * L) / (2.0 * R)
        # TODO: set nos joints:
        # self.session.setJointTargetVelocity(self.left_joint, v_l)
        # self.session.setJointTargetVelocity(self.right_joint, v_r)
        pass

    def _pose(self) -> Tuple[float, float, float]:
        """Retorne (x,y,yaw) do robô em metros/radianos."""
        # TODO: ler pose 2D do robô
        if self.last_position is None:
            self.last_position = np.array([0.0, 0.0])
        yaw = 0.0
        return float(self.last_position[0]), float(self.last_position[1]), float(yaw)

    def reset(self, seed: Optional[int] = None) -> np.ndarray:
        if seed is not None:
            set_seed(seed)
        self._connect()
        self._reset_sim()
        self.collision = False
        self.step_count = 0
        self.last_position = np.array([0.0, 0.0], dtype=np.float32)  # TODO: settar pose inicial correta
        obs = self._read_sensors()
        return obs

    def step(self, action_idx: int):
        self.step_count += 1
        alpha_v, alpha_w = self.actions[action_idx]
        v = alpha_v * self.v_lin
        w = alpha_w * self.w_ang
        self._set_wheel_speeds(v, w)

        time.sleep(0.05)

        obs = self._read_sensors()

        # 1) Penalizar proximidade média a obstáculos
        proximity = float(np.mean(obs))  # 0 longe, 1 perto
        r_obst = -2.0 * proximity

        # 2) Recompensar ir pra frente (maior alpha_v) e orientar-se com baixa rotação
        r_drive = +0.2 * alpha_v - 0.05 * (abs(alpha_w))

        # 3) Pequeno custo por passo (incentiva eficiência)
        r_step = -0.01

        # 4) Penalidade/terminação por colisão
        done = False
        r_collision = 0.0
        if self._collided():
            done = True
            r_collision = -5.0

        # 5) Limite de passos
        if self.step_count >= self.max_steps:
            done = True

        reward = r_obst + r_drive + r_step + r_collision
        info = {"proximity": proximity}
        return obs, float(reward), done, info

    @property
    def observation_space(self):
        return (self.sensor_dim,)

    @property
    def action_space_n(self):
        return self.n_actions


class RolloutBuffer:
    def __init__(self, size: int, obs_dim: int, gamma: float = 0.99, lam: float = 0.95):
        self.size = size
        self.obs = np.zeros((size, obs_dim), dtype=np.float32)
        self.actions = np.zeros(size, dtype=np.int64)
        self.rewards = np.zeros(size, dtype=np.float32)
        self.dones = np.zeros(size, dtype=np.bool_)
        self.values = np.zeros(size, dtype=np.float32)
        self.logprobs = np.zeros(size, dtype=np.float32)
        self.adv = np.zeros(size, dtype=np.float32)
        self.returns = np.zeros(size, dtype=np.float32)
        self.gamma = gamma
        self.lam = lam
        self.ptr = 0

    def add(self, obs, action, reward, done, value, logprob):
        self.obs[self.ptr] = obs
        self.actions[self.ptr] = action
        self.rewards[self.ptr] = reward
        self.dones[self.ptr] = done
        self.values[self.ptr] = value
        self.logprobs[self.ptr] = logprob
        self.ptr += 1

    def compute_advantages(self, last_value: float, last_done: bool):
        adv = 0.0
        for t in reversed(range(self.ptr)):
            next_non_terminal = 1.0 - float(self.dones[t] if t < self.ptr - 1 else last_done)
            next_value = self.values[t + 1] if t < self.ptr - 1 else last_value
            delta = self.rewards[t] + self.gamma * next_value * next_non_terminal - self.values[t]
            adv = delta + self.gamma * self.lam * next_non_terminal * adv
            self.adv[t] = adv
        self.returns[:self.ptr] = self.adv[:self.ptr] + self.values[:self.ptr]

    def get(self, batch_size: int):
        idxs = np.arange(self.ptr)
        np.random.shuffle(idxs)
        for start in range(0, self.ptr, batch_size):
            end = start + batch_size
            mb_idx = idxs[start:end]
            yield (
                torch.as_tensor(self.obs[mb_idx], dtype=torch.float32, device=DEVICE),
                torch.as_tensor(self.actions[mb_idx], dtype=torch.long, device=DEVICE),
                torch.as_tensor(self.logprobs[mb_idx], dtype=torch.float32, device=DEVICE),
                torch.as_tensor(self.adv[mb_idx], dtype=torch.float32, device=DEVICE),
                torch.as_tensor(self.returns[mb_idx], dtype=torch.float32, device=DEVICE),
            )
        self.ptr = 0

class ActorCritic(nn.Module):
    def __init__(self, obs_dim: int, n_actions: int, hidden_sizes=(128, 128)):
        super().__init__()
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, hidden_sizes[0]),
            nn.Tanh(),
            nn.Linear(hidden_sizes[0], hidden_sizes[1]),
            nn.Tanh(),
            nn.Linear(hidden_sizes[1], n_actions),
        )
        self.critic = nn.Sequential(
            nn.Linear(obs_dim, hidden_sizes[0]),
            nn.Tanh(),
            nn.Linear(hidden_sizes[0], hidden_sizes[1]),
            nn.Tanh(),
            nn.Linear(hidden_sizes[1], 1),
        )

    def forward(self, x):
        raise NotImplementedError

    def act(self, obs: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        logits = self.actor(obs)
        dist = torch.distributions.Categorical(logits=logits)
        action = dist.sample()
        logprob = dist.log_prob(action)
        value = self.critic(obs).squeeze(-1)
        return action, logprob, value

    def evaluate_actions(self, obs: torch.Tensor, actions: torch.Tensor):
        logits = self.actor(obs)
        dist = torch.distributions.Categorical(logits=logits)
        logprobs = dist.log_prob(actions)
        entropy = dist.entropy()
        values = self.critic(obs).squeeze(-1)
        return logprobs, entropy, values

@dataclass
class PPOConfig:
    total_steps: int = 50_000
    rollout_length: int = 1024
    learning_rate: float = 3e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_coef: float = 0.2
    update_epochs: int = 4
    minibatch_size: int = 256
    vf_coef: float = 0.5
    ent_coef: float = 0.01
    max_grad_norm: float = 0.5
    seed: int = 42
    log_interval: int = 1024


class PPOTrainer:
    def __init__(self, env: CoppeliaSimObstacleAvoidEnv, cfg: PPOConfig):
        self.env = env
        self.cfg = cfg
        set_seed(cfg.seed)

        obs_dim = env.observation_space[0]
        n_actions = env.action_space_n

        self.net = ActorCritic(obs_dim, n_actions).to(DEVICE)
        self.opt = optim.Adam(self.net.parameters(), lr=cfg.learning_rate)
        self.buf = RolloutBuffer(cfg.rollout_length, obs_dim, cfg.gamma, cfg.gae_lambda)

    @torch.no_grad()
    def _obs_to_t(self, obs: np.ndarray) -> torch.Tensor:
        return torch.as_tensor(obs, dtype=torch.float32, device=DEVICE).unsqueeze(0)

    def collect_rollout(self, obs: np.ndarray) -> Tuple[np.ndarray, float, bool]:
        for _ in range(self.cfg.rollout_length):
            obs_t = self._obs_to_t(obs)
            action_t, logprob_t, value_t = self.net.act(obs_t)
            action = int(action_t.item())
            logprob = float(logprob_t.item())
            value = float(value_t.item())

            next_obs, reward, done, info = self.env.step(action)
            self.buf.add(obs, action, reward, done, value, logprob)
            obs = next_obs

            if done:
                obs = self.env.reset()

        # bootstrap: último valor
        with torch.no_grad():
            last_value = self.net.critic(self._obs_to_t(obs)).item()
        # no último done da rollout: considere False, pois já fizemos resets internos
        self.buf.compute_advantages(last_value=last_value, last_done=False)
        return obs, 0.0, False

    def update(self):
        for _ in range(self.cfg.update_epochs):
            for mb_obs, mb_actions, mb_logprobs_old, mb_adv, mb_returns in self.buf.get(self.cfg.minibatch_size):
                # normalize advantages
                mb_adv = (mb_adv - mb_adv.mean()) / (mb_adv.std() + 1e-8)

                logprobs, entropy, values = self.net.evaluate_actions(mb_obs, mb_actions)
                ratio = torch.exp(logprobs - mb_logprobs_old)

                unclipped = ratio * mb_adv
                clipped = torch.clamp(ratio, 1.0 - self.cfg.clip_coef, 1.0 + self.cfg.clip_coef) * mb_adv
                policy_loss = -torch.min(unclipped, clipped).mean()

                # value loss (MSE)
                value_loss = 0.5 * (mb_returns - values).pow(2).mean()

                # entropy bonus
                entropy_loss = -entropy.mean()

                loss = policy_loss + self.cfg.vf_coef * value_loss + self.cfg.ent_coef * (-entropy_loss)

                self.opt.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(self.net.parameters(), self.cfg.max_grad_norm)
                self.opt.step()

    def train(self):
        obs = self.env.reset(self.cfg.seed)
        total_steps = 0
        last_log = 0

        while total_steps < self.cfg.total_steps:
            start_steps = total_steps
            obs, _, _ = self.collect_rollout(obs)
            total_steps += (self.cfg.rollout_length)

            self.update()

            if total_steps - last_log >= self.cfg.log_interval:
                print(f"[PPO] steps={total_steps} (+{total_steps - start_steps})")
                last_log = total_steps