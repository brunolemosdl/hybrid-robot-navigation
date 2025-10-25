import numpy as np

from collections import deque
from typing import List, Tuple

from .base import BasePlanner

from utils.map import is_free_pixel
from utils.conversions import world_to_pixel, pixel_to_world

from utils.logger import get_logger


class WavefrontPlanner(BasePlanner):
    def __init__(self, connectivity: int = 8):
        self.connectivity = connectivity
        self.logger = get_logger("wavefront")

    def plan(
        self,
        binary_map: np.ndarray,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        world_bounds: Tuple[float, float, float, float],
        resolution: Tuple[int, int],
    ) -> List[Tuple[float, float]]:
        self.logger.info("Starting Wavefront path planning")
        self.logger.debug(f"Parameters: connectivity={self.connectivity}")

        min_x, max_x, min_y, max_y = world_bounds
        image_width, image_height = resolution

        start_pixel = world_to_pixel(
            start_world, min_x, max_x, min_y, max_y, image_width, image_height
        )
        goal_pixel = world_to_pixel(
            goal_world, min_x, max_x, min_y, max_y, image_width, image_height
        )
        sx, sy = int(start_pixel[0]), int(start_pixel[1])
        gx, gy = int(goal_pixel[0]), int(goal_pixel[1])

        self.logger.debug(f"Start pixel: ({sx}, {sy}), Goal pixel: ({gx}, {gy})")

        if not is_free_pixel(binary_map, sx, sy) or not is_free_pixel(
            binary_map, gx, gy
        ):
            self.logger.error("Start or goal position is in collision")
            return []

        infinity = np.inf
        cost_map = np.full_like(binary_map, infinity, dtype=float)
        cost_map[binary_map == 255] = infinity
        cost_map[gy, gx] = 0.0

        queue = deque([(gx, gy)])

        if self.connectivity == 8:
            neighbors = [
                (-1, -1),
                (-1, 0),
                (-1, 1),
                (0, -1),
                (0, 1),
                (1, -1),
                (1, 0),
                (1, 1),
            ]
        else:
            neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        self.logger.info("Building wavefront cost map")
        processed_cells = 0
        while queue:
            if processed_cells % 1000 == 0 and processed_cells > 0:
                self.logger.debug(
                    f"Processed {processed_cells} cells, queue size: {len(queue)}"
                )

            x, y = queue.popleft()
            current_cost = cost_map[y, x]
            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= image_width or ny >= image_height:
                    continue
                if binary_map[ny, nx] == 255:
                    continue
                step_cost = np.hypot(dx, dy)
                new_cost = current_cost + step_cost
                if new_cost < cost_map[ny, nx]:
                    cost_map[ny, nx] = new_cost
                    queue.append((nx, ny))
            processed_cells += 1

        if np.isinf(cost_map[sy, sx]):
            self.logger.warning("No path found - start position unreachable")
            return []

        self.logger.debug(f"Cost map built, start cost: {cost_map[sy, sx]:.2f}")
        self.logger.info("Backtracking to find path")

        path_pixels = [(sx, sy)]
        x, y = sx, sy

        for step in range(image_width * image_height):
            if step % 100 == 0:
                self.logger.debug(
                    f"Backtracking step {step}, current position: ({x}, {y})"
                )

            if (x, y) == (gx, gy):
                self.logger.info(f"Goal reached after {step} backtracking steps")
                break
            min_cost = cost_map[y, x]
            next_cell = None
            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= image_width or ny >= image_height:
                    continue
                c = cost_map[ny, nx]
                if c < min_cost:
                    min_cost = c
                    next_cell = (nx, ny)
            if next_cell is None:
                self.logger.warning("Backtracking failed - no valid next cell")
                break
            x, y = next_cell
            path_pixels.append((x, y))

        self.logger.info(f"Path found with {len(path_pixels)} waypoints")
        return [
            pixel_to_world(p, min_x, max_x, min_y, max_y, image_width, image_height)
            for p in path_pixels
        ]
