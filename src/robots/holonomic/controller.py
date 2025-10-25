import math
from dataclasses import dataclass
from typing import Tuple

from robots.base.controller import ControlOutput


@dataclass
class HolonomicGains:
    kp_position: float = 1.2
    kp_orientation: float = 2.0
    max_linear_speed: float = 0.6
    max_angular_speed: float = 2.0


class HolonomicController:
    def __init__(self, gains: HolonomicGains | None = None) -> None:
        self.gains = gains or HolonomicGains()

    def compute_control(
        self,
        current_pose: Tuple[float, float, float],
        target_pose: Tuple[float, float, float],
    ) -> ControlOutput:
        cx, cy, ctheta = current_pose
        tx, ty, ttheta = target_pose

        ex = tx - cx
        ey = ty - cy
        e_theta = self._wrap_angle(ttheta - ctheta)

        vx = self.gains.kp_position * ex
        vy = self.gains.kp_position * ey
        wz = self.gains.kp_orientation * e_theta

        speed = math.hypot(vx, vy)
        if speed > self.gains.max_linear_speed:
            scale = self.gains.max_linear_speed / max(speed, 1e-9)
            vx *= scale
            vy *= scale

        if abs(wz) > self.gains.max_angular_speed:
            wz = math.copysign(self.gains.max_angular_speed, wz)

        return ControlOutput(
            linear_velocity_x=vx, linear_velocity_y=vy, angular_velocity_z=wz
        )

    def _wrap_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
