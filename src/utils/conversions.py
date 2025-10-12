import math
import networkx as nx

from typing import Tuple, Optional, List


def world_to_pixel(
    world_position: Tuple[float, float],
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    width: int,
    height: int,
) -> Tuple[int, int]:
    world_x, world_y = world_position

    normalized_x = (world_x - min_x) / (max_x - min_x)
    normalized_y = (world_y - min_y) / (max_y - min_y)

    pixel_x = int(normalized_x * (width - 1))
    pixel_y = int((1 - normalized_y) * (height - 1))

    return pixel_x, pixel_y


def pixel_to_world(
    pixel_position: Tuple[int, int],
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    width: int,
    height: int,
) -> Tuple[float, float]:
    pixel_x, pixel_y = pixel_position

    normalized_x = pixel_x / (width - 1)
    normalized_y = 1 - (pixel_y / (height - 1))

    world_x = min_x + normalized_x * (max_x - min_x)
    world_y = min_y + normalized_y * (max_y - min_y)

    return world_x, world_y


def find_nearest_free_node(
    graph: nx.Graph, target_pixel: Tuple[int, int]
) -> Optional[Tuple[int, int]]:
    if not graph.nodes():
        return None

    minimum_distance = float("inf")
    nearest_node = None

    for node_x, node_y in graph.nodes():
        current_distance = math.dist((node_x, node_y), target_pixel)

        if current_distance < minimum_distance:
            minimum_distance = current_distance
            nearest_node = (node_x, node_y)

    return nearest_node


def find_path_a_star(
    graph: nx.Graph, start: Tuple[int, int], goal: Tuple[int, int]
) -> Optional[List[Tuple[int, int]]]:
    try:
        return nx.astar_path(
            graph,
            start,
            goal,
            heuristic=lambda node_a, node_b: math.dist(node_a, node_b),
            weight="weight",
        )
    except nx.NetworkXNoPath:
        return None


def quaternion_to_euler(
    x: float, y: float, z: float, w: float
) -> Tuple[float, float, float]:
    roll_term_1 = +2.0 * (w * x + y * z)
    roll_term_2 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(roll_term_1, roll_term_2)

    pitch_term = +2.0 * (w * y - z * x)
    pitch_term = +1.0 if pitch_term > +1.0 else pitch_term
    pitch_term = -1.0 if pitch_term < -1.0 else pitch_term
    pitch = math.asin(pitch_term)

    yaw_term_1 = +2.0 * (w * z + x * y)
    yaw_term_2 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(yaw_term_1, yaw_term_2)

    return roll, pitch, yaw
