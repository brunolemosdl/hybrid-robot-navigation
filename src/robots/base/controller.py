from dataclasses import dataclass
from typing import Protocol, Tuple


@dataclass
class ControlOutput:
    linear_velocity_x: float = 0.0
    linear_velocity_y: float = 0.0
    angular_velocity_z: float = 0.0


class BaseController(Protocol):
    def compute_control(
        self,
        current_pose: Tuple[float, float, float],
        target_pose: Tuple[float, float, float],
    ) -> ControlOutput: ...
