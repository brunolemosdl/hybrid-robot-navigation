import numpy as np

from typing import List, Tuple

from .base import BasePlanner

from utils.conversions import (
    world_to_pixel,
    pixel_to_world,
    find_nearest_free_node,
    find_path_a_star,
)
from utils.map import create_graph_from_binary_image

from utils.logger import get_logger


class RoadmapPlanner(BasePlanner):
    def __init__(self, sample_rate: int = 5):
        self.sample_rate = sample_rate
        self.logger = get_logger("roadmap")

    def plan(
        self,
        binary_map: np.ndarray,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        world_bounds: Tuple[float, float, float, float],
        resolution: Tuple[int, int],
    ) -> List[Tuple[float, float]]:
        self.logger.info("Starting Roadmap path planning")
        self.logger.debug(f"Parameters: sample_rate={self.sample_rate}")

        min_x, max_x, min_y, max_y = world_bounds
        image_width, image_height = resolution

        self.logger.debug("Creating graph from binary image")
        graph = create_graph_from_binary_image(binary_map, self.sample_rate)
        self.logger.debug(
            f"Graph created with {len(graph.nodes)} nodes and {len(graph.edges)} edges"
        )

        start_pixel = world_to_pixel(
            start_world, min_x, max_x, min_y, max_y, image_width, image_height
        )
        goal_pixel = world_to_pixel(
            goal_world, min_x, max_x, min_y, max_y, image_width, image_height
        )

        self.logger.debug(f"Start pixel: {start_pixel}, Goal pixel: {goal_pixel}")

        self.logger.debug("Finding nearest free nodes")
        start_node = find_nearest_free_node(graph, start_pixel)
        goal_node = find_nearest_free_node(graph, goal_pixel)

        if not start_node or not goal_node:
            self.logger.error("Could not find valid start or goal nodes in graph")
            return []

        self.logger.debug(f"Start node: {start_node}, Goal node: {goal_node}")
        self.logger.info("Searching path using A*")

        path_pixels = find_path_a_star(graph, start_node, goal_node)
        if not path_pixels:
            self.logger.warning("No path found using A* search")
            return []

        self.logger.info(f"Path found with {len(path_pixels)} waypoints")
        return [
            pixel_to_world(p, min_x, max_x, min_y, max_y, image_width, image_height)
            for p in path_pixels
        ]
