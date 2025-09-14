from dataclasses import dataclass
from typing import Dict, Any, Tuple
from src.core import Pose2D, CoppeliaSimSession
from src.robots.epuck import (
    find_epuck,
    find_wheel_joints,
    set_wheel_speeds,
    disable_child_script,
)
from src.robots.diff_drive import diff_drive


@dataclass
class EnvConfig:
    dt: float = 0.05
    v_max: float = 0.10
    w_max: float = 2.0
    target: Tuple[float, float] = (0.5, 0.0)


class EpuckNavV0:
    def __init__(self, session: CoppeliaSimSession, cfg: EnvConfig = EnvConfig()):
        self.sess = session
        self.cfg = cfg
        self.epuck = find_epuck(session)
        disable_child_script(session, self.epuck)
        self.left, self.right = find_wheel_joints(session, self.epuck)

    def reset(self) -> Dict[str, Any]:
        # aqui você pode reposicionar o robô, se quiser
        return self._obs()

    def step(self, action: Tuple[float, float]):
        v = max(min(action[0], self.cfg.v_max), -self.cfg.v_max)
        w = max(min(action[1], self.cfg.w_max), -self.cfg.w_max)
        wl, wr = diff_drive(v, w)
        set_wheel_speeds(self.sess, self.left, self.right, wl, wr)
        # integra por um passo
        self.sess.step()
        obs = self._obs()
        reward = self._reward(obs)
        done = False
        info = {}
        return obs, reward, done, info

    def _obs(self) -> Dict[str, Any]:
        pose = self.sess.get_pose2d_world(self.epuck)
        return {"pose": pose}

    def _reward(self, obs: Dict[str, Any]) -> float:
        px, py = obs["pose"].x, obs["pose"].y
        tx, ty = self.cfg.target
        dist = ((px - tx) ** 2 + (py - ty) ** 2) ** 0.5
        return -dist  # simples: quanto mais perto, melhor
