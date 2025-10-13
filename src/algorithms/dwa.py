from dataclasses import dataclass
from typing import Tuple, Iterable, Optional
import numpy as np


@dataclass
class DWAConfig:
    dt: float = 0.1
    predict_time: float = 2.0

    v_min: float = 0.0
    v_max: float = 1.0
    w_min: float = -1.5
    w_max: float = 1.5

    a_v: float = 0.8
    a_w: float = 2.0

    v_reso: float = 0.05
    w_reso: float = 0.05

    # Geometria/segurança
    robot_radius: float = 0.20
    obstacle_inflation: float = 0.05

    # Pesos de custo
    w_clearance: float = 1.5
    w_speed: float = 0.5


class DWA:
    def __init__(self, config: Optional[DWAConfig] = None):
        self.cfg = config or DWAConfig()
        self.last_u = np.array([0.0, 0.0], dtype=float)

    def plan(
        self,
        x: np.ndarray,
        goal_xy: Iterable[float],
        obstacles: Optional[np.ndarray] = None,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Decide (v,w) ótimo e retorna (u, traj, custo_min).
        - x: np.array([x, y, theta, v, w])
        - goal_xy: (gx, gy)
        - obstacles: array Nx2 (x,y) ou Nx3 (x,y,r)
        """
        v_low, v_high, w_low, w_high = self._dynamic_window(x)

        best_u = np.array([0.0, 0.0], dtype=float)
        best_traj = np.asarray([x], dtype=float)
        min_cost = np.inf

        v_samples = np.arange(v_low, v_high + 1e-9, self.cfg.v_reso)
        w_samples = np.arange(w_low, w_high + 1e-9, self.cfg.w_reso)

        for v in v_samples:
            for w in w_samples:
                u = np.array([v, w], dtype=float)
                traj = self._simulate(x, u)

                g = self._goal_cost(traj, goal_xy)
                c = self._clearance_cost(traj, obstacles)
                s = self._speed_reward(u)

                cost = self.cfg.w_goal * g + self.cfg.w_clearance * c - self.cfg.w_speed * s

                if cost < min_cost:
                    min_cost = cost
                    best_u = u
                    best_traj = traj

        self.last_u = best_u
        return best_u, best_traj, min_cost

    def _motion(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """Modelo uniciclo: aplica (v,w) por dt."""
        x_next = x.copy()
        v, w = u
        x_next[0] += v * np.cos(x[2]) * self.cfg.dt
        x_next[1] += v * np.sin(x[2]) * self.cfg.dt
        x_next[2] += w * self.cfg.dt
        x_next[3] = v
        x_next[4] = w
        return x_next

    def _dynamic_window(self, x: np.ndarray) -> Tuple[float, float, float, float]:
        v, w = float(x[3]), float(x[4])

        v_low  = max(self.cfg.v_min, v - self.cfg.a_v * self.cfg.dt)
        v_high = min(self.cfg.v_max, v + self.cfg.a_v * self.cfg.dt)
        w_low  = max(self.cfg.w_min, w - self.cfg.a_w * self.cfg.dt)
        w_high = min(self.cfg.w_max, w + self.cfg.a_w * self.cfg.dt)

        return v_low, v_high, w_low, w_high

    def _simulate(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        traj = [x.copy()]
        t = 0.0
        while t <= self.cfg.predict_time:
            x = self._motion(x, u)
            traj.append(x.copy())
            t += self.cfg.dt
        return np.asarray(traj, dtype=float)

    def _clearance_cost(self, traj: np.ndarray, obstacles: Optional[np.ndarray]) -> float:
        """
        Retorna custo de folga (maior = pior). Colisão -> custo enorme.
        obstacles: Nx2 (x,y) ou Nx3 (x,y,r)
        """
        if obstacles is None or len(obstacles) == 0:
            return 0.0

        traj_xy = traj[:, :2]
        min_dist = np.inf

        obstacles = np.asarray(obstacles, dtype=float)
        if obstacles.shape[1] == 2:
            obstacles = np.hstack([obstacles, np.zeros((obstacles.shape[0], 1))])

        for ox, oy, r in obstacles:
            d = np.linalg.norm(traj_xy - np.array([ox, oy]), axis=1) - (
                r + self.cfg.robot_radius + self.cfg.obstacle_inflation
            )
            if np.any(d < 0.0):  # colisão
                return 1e9
            local_min = float(np.min(d))
            if local_min < min_dist:
                min_dist = local_min

        return 1.0 / (min_dist + 1e-6)

    def _goal_cost(self, traj: np.ndarray, goal_xy: Iterable[float]) -> float:
        last_xy = traj[-1, :2]
        goal_xy = np.asarray(goal_xy, dtype=float)
        return float(np.linalg.norm(last_xy - goal_xy))

    def _speed_reward(self, u: np.ndarray) -> float:
        v = float(u[0])
        return v  # usamos como recompensa e subtraímos no custo total
