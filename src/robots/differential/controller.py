import math
from dataclasses import dataclass
from typing import Tuple

from robots.base.controller import ControlOutput


@dataclass
class DifferentialGains:
    kp_linear: float = 1.0
    kp_angular: float = 2.0
    max_linear_speed: float = 0.4
    max_angular_speed: float = 1.8
    goal_tolerance: float = 0.03
    angle_tolerance: float = 0.15
    slowdown_angle: float = 0.8


class DifferentialController:
    def __init__(self, gains: DifferentialGains | None = None) -> None:
        self.gains = gains or DifferentialGains()

    def compute_control(
        self,
        current_pose: Tuple[float, float, float],
        target_pose: Tuple[float, float, float],
    ) -> ControlOutput:
        cx, cy, ctheta = current_pose
        tx, ty, _ = target_pose
        dx, dy = tx - cx, ty - cy

        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        heading_error = self._wrap_angle(target_angle - ctheta)

        if distance < self.gains.goal_tolerance:
            return ControlOutput()

        angular_velocity = max(
            -self.gains.max_angular_speed,
            min(self.gains.max_angular_speed, self.gains.kp_angular * heading_error),
        )

        base_linear = self.gains.kp_linear * distance
        if abs(heading_error) >= math.pi / 2:
            scale = 0.0
        else:
            cos_term = max(0.0, math.cos(heading_error))
            ramp = max(
                0.0, 1.0 - abs(heading_error) / max(self.gains.slowdown_angle, 1e-6)
            )
            scale = max(0.0, min(1.0, 0.5 * cos_term + 0.5 * ramp))

        linear_velocity = max(
            -self.gains.max_linear_speed,
            min(self.gains.max_linear_speed, base_linear * scale),
        )

        return ControlOutput(
            linear_velocity_x=linear_velocity, angular_velocity_z=angular_velocity
        )

    def _wrap_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
