import numpy as np

from typing import List, Tuple


class BasePlanner:
    def plan(
        self,
        binary_map: np.ndarray,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        world_bounds: Tuple[float, float, float, float],
        resolution: Tuple[int, int],
    ) -> List[Tuple[float, float]]:
        raise NotImplementedError("Planner must implement plan")
