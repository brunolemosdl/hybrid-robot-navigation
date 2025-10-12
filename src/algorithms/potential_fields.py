import numpy as np

from typing import List, Tuple

from .base import BasePlanner

from utils.map import is_free_pixel
from utils.geometry import euclidean_distance
from utils.conversions import world_to_pixel, pixel_to_world

from utils.logger import get_logger


class PotentialFieldsPlanner(BasePlanner):
    def __init__(
        self,
        attractive_gain: float = 1.0,
        repulsive_gain: float = 5.0,
        step_size_meters: float = 0.05,
        maximum_iterations: int = 2000,
        goal_distance_threshold: float = 0.05,
        minimum_obstacle_distance: float = 0.2,
        noise_gain: float = 0.2,
        stuck_iterations: int = 25,
    ):
        self.attractive_gain = attractive_gain
        self.repulsive_gain = repulsive_gain
        self.step_size_meters = step_size_meters
        self.maximum_iterations = maximum_iterations
        self.goal_distance_threshold = goal_distance_threshold
        self.minimum_obstacle_distance = minimum_obstacle_distance
        self.noise_gain = noise_gain
        self.stuck_iterations = stuck_iterations
        self.logger = get_logger("potential_fields")

    def plan(
        self,
        binary_map: np.ndarray,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        world_bounds: Tuple[float, float, float, float],
        resolution: Tuple[int, int],
    ) -> List[Tuple[float, float]]:
        self.logger.info("Starting Potential Fields path planning")
        self.logger.debug(
            f"Parameters: attractive_gain={self.attractive_gain}, repulsive_gain={self.repulsive_gain}, step_size={self.step_size_meters}"
        )

        min_x, max_x, min_y, max_y = world_bounds
        image_width, image_height = resolution

        start_pixel = world_to_pixel(
            start_world, min_x, max_x, min_y, max_y, image_width, image_height
        )
        goal_pixel = world_to_pixel(
            goal_world, min_x, max_x, min_y, max_y, image_width, image_height
        )

        self.logger.debug(f"Start pixel: {start_pixel}, Goal pixel: {goal_pixel}")

        if not is_free_pixel(binary_map, *start_pixel) or not is_free_pixel(
            binary_map, *goal_pixel
        ):
            self.logger.error("Start or goal position is in collision")
            return []

        start = np.array(start_world, dtype=float)
        goal = np.array(goal_world, dtype=float)
        position = start.copy()

        path: List[Tuple[float, float]] = []

        previous_distance = euclidean_distance(position, goal)
        stuck_counter = 0

        self.logger.info("Starting potential fields navigation")
        for iteration in range(self.maximum_iterations):
            if iteration % 200 == 0:
                self.logger.debug(
                    f"PF iteration {iteration}/{self.maximum_iterations}, distance to goal: {euclidean_distance(position, goal):.4f}"
                )

            path.append(tuple(position))

            distance_to_goal = euclidean_distance(position, goal)
            if distance_to_goal < self.goal_distance_threshold:
                self.logger.info(f"Goal reached after {iteration} iterations")
                break

            attractive_force = self._attractive_force(position, goal)
            repulsive_force = self._repulsive_force(
                position, binary_map, world_bounds, resolution
            )
            total_force = attractive_force + repulsive_force

            if abs(previous_distance - distance_to_goal) < 1e-3:
                stuck_counter += 1
            else:
                stuck_counter = 0
            previous_distance = distance_to_goal

            if stuck_counter > self.stuck_iterations:
                self.logger.debug(
                    f"Robot stuck, applying noise at iteration {iteration}"
                )
                noise = np.random.uniform(-1, 1, 2)
                noise /= np.linalg.norm(noise)
                total_force += self.noise_gain * noise
                stuck_counter = 0

            norm = np.linalg.norm(total_force)
            if norm < 1e-9:
                self.logger.warning("Force magnitude too small, stopping")
                break

            position = position + (total_force / norm) * self.step_size_meters

            px, py = world_to_pixel(
                tuple(position), min_x, max_x, min_y, max_y, image_width, image_height
            )
            if not is_free_pixel(binary_map, px, py):
                self.logger.warning("Collision detected, stopping")
                break

        if euclidean_distance(position, goal) >= self.goal_distance_threshold:
            self.logger.warning(
                f"Failed to reach goal after {self.maximum_iterations} iterations"
            )
            return []

        self.logger.info(f"Path found with {len(path)} waypoints")
        return [
            pixel_to_world(
                world_to_pixel(
                    p, min_x, max_x, min_y, max_y, image_width, image_height
                ),
                min_x,
                max_x,
                min_y,
                max_y,
                image_width,
                image_height,
            )
            for p in path
        ]

    def _attractive_force(self, position: np.ndarray, goal: np.ndarray) -> np.ndarray:
        return self.attractive_gain * (goal - position)

    def _repulsive_force(
        self,
        position: np.ndarray,
        binary_map: np.ndarray,
        world_bounds: Tuple[float, float, float, float],
        resolution: Tuple[int, int],
    ) -> np.ndarray:
        min_x, max_x, min_y, max_y = world_bounds
        image_width, image_height = resolution

        px, py = world_to_pixel(
            tuple(position), min_x, max_x, min_y, max_y, image_width, image_height
        )
        px, py = int(px), int(py)

        force = np.zeros(2, dtype=float)
        search_radius = 6
        threshold_distance = self.minimum_obstacle_distance

        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                cx, cy = px + dx, py + dy
                if cx < 0 or cy < 0 or cx >= image_width or cy >= image_height:
                    continue
                if binary_map[cy, cx] != 255:
                    continue

                obstacle_world = np.array(
                    pixel_to_world(
                        (cx, cy), min_x, max_x, min_y, max_y, image_width, image_height
                    )
                )
                difference = position - obstacle_world
                distance = np.linalg.norm(difference)
                if 0 < distance < threshold_distance:
                    direction = difference / distance
                    magnitude = (
                        self.repulsive_gain
                        * (1.0 / distance - 1.0 / threshold_distance)
                        / (distance**2)
                    )
                    force += magnitude * direction

        return force
