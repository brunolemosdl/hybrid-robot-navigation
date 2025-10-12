import random
import numpy as np

from typing import List, Tuple, Optional

from .base import BasePlanner

from utils.geometry import euclidean_distance
from utils.map import is_free_pixel, is_segment_free
from utils.conversions import world_to_pixel, pixel_to_world

from utils.logger import get_logger


class TreeNode:
    def __init__(
        self,
        position: np.ndarray,
        parent: Optional["TreeNode"] = None,
        cost: float = 0.0,
    ):
        self.position = position
        self.parent = parent
        self.cost = cost


class RRTPlanner(BasePlanner):
    def __init__(
        self,
        step_size_pixels: float = 4.0,
        maximum_iterations: int = 50000,
        goal_sample_probability: float = 0.15,
        goal_threshold_pixels: float = 3.0,
        maximum_extend_steps: int = 32,
    ):
        self.step_size_pixels = step_size_pixels
        self.maximum_iterations = maximum_iterations
        self.goal_sample_probability = goal_sample_probability
        self.goal_threshold_pixels = goal_threshold_pixels
        self.maximum_extend_steps = maximum_extend_steps
        self.logger = get_logger("rrt")

    def plan(
        self,
        binary_map: np.ndarray,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        world_bounds: Tuple[float, float, float, float],
        resolution: Tuple[int, int],
    ) -> List[Tuple[float, float]]:
        self.logger.info("Starting RRT path planning")
        self.logger.debug(
            f"Parameters: step_size={self.step_size_pixels}, max_iter={self.maximum_iterations}, goal_prob={self.goal_sample_probability}"
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

        start_node = TreeNode(np.array(start_pixel, dtype=float))
        goal_node = TreeNode(np.array(goal_pixel, dtype=float))

        tree_front, tree_back = [start_node], [goal_node]

        if is_segment_free(binary_map, tuple(start_pixel), tuple(goal_pixel)):
            self.logger.info("Direct path found between start and goal")
            path_pixels = [tuple(start_pixel), tuple(goal_pixel)]
            return [
                pixel_to_world(p, min_x, max_x, min_y, max_y, image_width, image_height)
                for p in path_pixels
            ]

        self.logger.info("Starting bidirectional RRT search")
        for iteration in range(self.maximum_iterations):
            if iteration % 1000 == 0:
                self.logger.debug(
                    f"RRT iteration {iteration}/{self.maximum_iterations}"
                )
                self.logger.debug(
                    f"Tree sizes: front={len(tree_front)}, back={len(tree_back)}"
                )

            sample = self._sample_free(
                binary_map, image_width, image_height, goal_pixel
            )

            status_front, new_front, _ = self._extend(binary_map, tree_front, sample)
            if status_front == "trapped":
                continue

            status_back, meet_node, _ = self._connect(
                binary_map, tree_back, new_front.position
            )
            if status_back == "reached" or (
                status_back == "advanced"
                and euclidean_distance(meet_node.position, new_front.position)
                <= self.goal_threshold_pixels
            ):
                self.logger.info(f"Path found after {iteration} iterations")
                self.logger.debug(f"Connection status: {status_back}")
                path = self._build_path_meet(
                    new_front,
                    meet_node,
                    min_x,
                    max_x,
                    min_y,
                    max_y,
                    image_width,
                    image_height,
                )
                self.logger.info(f"Final path has {len(path)} waypoints")
                return path

            tree_front, tree_back = tree_back, tree_front

        self.logger.warning(f"No path found after {self.maximum_iterations} iterations")
        return []

    def _sample_free(
        self,
        binary_map: np.ndarray,
        image_width: int,
        image_height: int,
        goal_pixel: Tuple[int, int],
    ) -> np.ndarray:
        if random.random() < self.goal_sample_probability:
            return np.array(goal_pixel, dtype=float)
        for _ in range(100):
            candidate = np.array(
                [
                    random.uniform(0, image_width - 1),
                    random.uniform(0, image_height - 1),
                ]
            )
            if is_free_pixel(binary_map, int(candidate[0]), int(candidate[1])):
                return candidate
        free_rows, free_cols = np.where(binary_map != 255)
        if len(free_cols) == 0:
            return np.array(goal_pixel, dtype=float)
        index = np.random.randint(0, len(free_cols))
        return np.array([free_cols[index], free_rows[index]], dtype=float)

    def _extend(self, binary_map: np.ndarray, tree: List[TreeNode], target: np.ndarray):
        nearest = min(tree, key=lambda n: euclidean_distance(n.position, target))
        new_position = self._steer_clamped(nearest.position, target, binary_map)
        if not is_segment_free(
            binary_map,
            tuple(nearest.position.astype(int)),
            tuple(new_position.astype(int)),
        ):
            return "trapped", None, []
        new_node = TreeNode(
            new_position,
            nearest,
            nearest.cost + euclidean_distance(nearest.position, new_position),
        )
        tree.append(new_node)

        steps = 1
        while (
            euclidean_distance(new_node.position, target) > self.goal_threshold_pixels
            and steps < self.maximum_extend_steps
        ):
            nearest = new_node
            next_position = self._steer_clamped(nearest.position, target, binary_map)
            if not is_segment_free(
                binary_map,
                tuple(nearest.position.astype(int)),
                tuple(next_position.astype(int)),
            ):
                break
            new_node = TreeNode(
                next_position,
                nearest,
                nearest.cost + euclidean_distance(nearest.position, next_position),
            )
            tree.append(new_node)
            steps += 1
        status = (
            "reached"
            if euclidean_distance(new_node.position, target)
            <= self.goal_threshold_pixels
            else "advanced"
        )
        return status, new_node, []

    def _connect(
        self, binary_map: np.ndarray, tree: List[TreeNode], target_position: np.ndarray
    ):
        status, last_node, _ = self._extend(binary_map, tree, target_position)
        return status, last_node, []

    def _steer_clamped(
        self, source: np.ndarray, target: np.ndarray, binary_map: np.ndarray
    ) -> np.ndarray:
        direction = target - source
        distance = np.linalg.norm(direction)
        if distance == 0:
            position = source
        elif distance <= self.step_size_pixels:
            position = target
        else:
            position = source + (direction / distance) * self.step_size_pixels
        image_height, image_width = binary_map.shape
        position[0] = np.clip(position[0], 0, image_width - 1)
        position[1] = np.clip(position[1], 0, image_height - 1)
        return position

    def _build_path_meet(
        self,
        node_front: TreeNode,
        node_back: TreeNode,
        min_x: float,
        max_x: float,
        min_y: float,
        max_y: float,
        image_width: int,
        image_height: int,
    ) -> List[Tuple[float, float]]:
        path_front, path_back = [], []
        node = node_front
        while node is not None:
            path_front.append(tuple(node.position))
            node = node.parent
        path_front.reverse()
        node = node_back
        while node is not None:
            path_back.append(tuple(node.position))
            node = node.parent
        if path_front and path_back and path_front[-1] == path_back[-1]:
            path_back = path_back[:-1]
        path_pixels = path_front + path_back
        return [
            pixel_to_world(p, min_x, max_x, min_y, max_y, image_width, image_height)
            for p in path_pixels
        ]
