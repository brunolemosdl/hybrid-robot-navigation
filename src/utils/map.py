import cv2
import math
import json
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt

from pathlib import Path
from typing import Tuple, Dict, Optional, List

from utils.session import get_coppelia_session
from utils.paths import resolve_output_path, ensure_directory_exists


def is_free_pixel(binary_map: np.ndarray, x: int, y: int) -> bool:
    if x < 0 or y < 0 or y >= binary_map.shape[0] or x >= binary_map.shape[1]:
        return False

    return binary_map[y, x] != 255


def is_segment_free(
    binary_map: np.ndarray, a: Tuple[int, int], b: Tuple[int, int]
) -> bool:
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy

    while True:
        if binary_map[y0, x0] == 255:
            return False

        if x0 == x1 and y0 == y1:
            break

        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx

        if e2 < dx:
            err += dx
            y0 += sy

    return True


def create_graph_from_binary_image(
    binary_image: np.ndarray, sample_rate: int = 1
) -> nx.Graph:
    graph = nx.Graph()
    height, width = binary_image.shape
    cell_size = max(1, sample_rate)
    free_cells: Dict[Tuple[int, int], Tuple[int, int]] = {}
    for y in range(0, height, cell_size):
        for x in range(0, width, cell_size):
            x2, y2 = min(x + cell_size, width), min(y + cell_size, height)
            cell = binary_image[y:y2, x:x2]
            if cell.size == 0 or np.any(cell != 0):
                continue
            cx, cy = x + (x2 - x) // 2, y + (y2 - y) // 2
            graph.add_node((cx, cy))
            free_cells[(cx, cy)] = (x // cell_size, y // cell_size)
    neighbor_dirs = [
        (-1, -1),
        (-1, 0),
        (-1, 1),
        (0, -1),
        (0, 1),
        (1, -1),
        (1, 0),
        (1, 1),
    ]
    index_to_center = {v: k for k, v in free_cells.items()}
    for center, (ix, iy) in free_cells.items():
        for dx, dy in neighbor_dirs:
            ncenter = index_to_center.get((ix + dx, iy + dy))
            if ncenter:
                graph.add_edge(
                    center,
                    ncenter,
                    weight=np.hypot(center[0] - ncenter[0], center[1] - ncenter[1]),
                )
    return graph


def extract_map_from_simulation(
    sensor_path: str = "/scene/floor/sensor",
    output_dir: str = "./results",
    selected_robot: str = "ePuck",
    clean_map: bool = True,
    save_map: bool = False,
    map_name: Optional[str] = None,
    scene: str = "default",
    robot_type: str = "differential",
) -> Optional[np.ndarray]:
    session = get_coppelia_session()
    sim = session.simulation

    session.start()
    session.wait_until_running()

    try:
        sensor_handle = sim.getObject(sensor_path)
        session.step()
        image_buffer, resolution = sim.getVisionSensorImg(sensor_handle)
        width, height = resolution

        image_array = np.frombuffer(image_buffer, dtype=np.uint8).reshape(
            height, width, 3
        )

        gray = cv2.cvtColor(image_array, cv2.COLOR_RGB2GRAY)
        _, binary_image = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)
        binary_image = cv2.flip(binary_image, 1)
        navigable_area = np.copy(binary_image)

        sel_handle = (
            sim.getObject("/ePuck")
            if selected_robot == "ePuck"
            else sim.getObject("/OmniPlatform")
        )
        sel_max_x = sim.getObjectFloatParameter(
            sel_handle, sim.objfloatparam_objbbox_max_x
        )[1]
        sel_min_x = sim.getObjectFloatParameter(
            sel_handle, sim.objfloatparam_objbbox_min_x
        )[1]
        sel_max_y = sim.getObjectFloatParameter(
            sel_handle, sim.objfloatparam_objbbox_max_y
        )[1]
        sel_min_y = sim.getObjectFloatParameter(
            sel_handle, sim.objfloatparam_objbbox_min_y
        )[1]
        robot_width_world = sel_max_x - sel_min_x
        robot_height_world = sel_max_y - sel_min_y
        robot_radius_world = max(robot_width_world, robot_height_world) / 2.0

        scene_floor_handle = sim.getObject("/scene/floor")
        max_x = sim.getObjectFloatParameter(
            scene_floor_handle, sim.objfloatparam_objbbox_max_x
        )[1]
        min_x = sim.getObjectFloatParameter(
            scene_floor_handle, sim.objfloatparam_objbbox_min_x
        )[1]
        max_y = sim.getObjectFloatParameter(
            scene_floor_handle, sim.objfloatparam_objbbox_max_y
        )[1]
        min_y = sim.getObjectFloatParameter(
            scene_floor_handle, sim.objfloatparam_objbbox_min_y
        )[1]

        pixels_per_meter_x = (width - 1) / (max_x - min_x)
        pixels_per_meter_y = (height - 1) / (max_y - min_y)
        pixels_per_meter = max(pixels_per_meter_x, pixels_per_meter_y)
        robot_radius_pixels = int(math.ceil(robot_radius_world * pixels_per_meter))
        kernel_size = max(1, 2 * robot_radius_pixels + 1)

        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        navigable_area = cv2.dilate(navigable_area, kernel, iterations=1)

        middle_y = height // 2
        for pixel_x in range(0, width // 2):
            if navigable_area[middle_y, pixel_x] == 255:
                navigable_area[:, :pixel_x] = 255
                break
        for pixel_x in range(width - 1, width // 2, -1):
            if navigable_area[middle_y, pixel_x] == 255:
                navigable_area[:, pixel_x:] = 255
                break

        session.stop()
        session.wait_until_stopped()

        if clean_map:
            kernel = np.ones((3, 3), np.uint8)
            navigable_area = cv2.morphologyEx(navigable_area, cv2.MORPH_CLOSE, kernel)

        if save_map:
            Path(output_dir).mkdir(parents=True, exist_ok=True)
            final_map_name = map_name if map_name else f"{scene}_{robot_type}_map.png"
            map_file = str(Path(output_dir) / final_map_name)
            cv2.imwrite(map_file, navigable_area)

        return navigable_area

    except Exception as e:
        try:
            session.stop()
            session.wait_until_stopped()
        except:
            pass
        return None


def visualize_map_quick(
    binary_map: np.ndarray,
    title: str = "Map",
    world_bounds: Tuple[float, float, float, float] = (-1.0, 1.0, -1.0, 1.0),
):
    try:
        min_x, max_x, min_y, max_y = world_bounds
        plt.figure(figsize=(6, 6))
        plt.imshow(
            binary_map, cmap="gray", extent=[min_x, max_x, min_y, max_y], origin="upper"
        )
        plt.title(title)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis("equal")
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    except Exception as e:
        pass


def visualize_map_with_path(
    binary_map: np.ndarray,
    path_world: List[Tuple[float, float]],
    world_bounds: Tuple[float, float, float, float],
    start_world: Tuple[float, float],
    goal_world: Tuple[float, float],
    title: str = "Map with Path",
):
    try:
        min_x, max_x, min_y, max_y = world_bounds
        plt.figure(figsize=(8, 8))
        plt.imshow(
            binary_map,
            cmap="gray",
            extent=[min_x, max_x, min_y, max_y],
            origin="upper",
            alpha=0.85,
        )
        if path_world:
            xs = [p[0] for p in path_world]
            ys = [p[1] for p in path_world]
            plt.plot(xs, ys, linewidth=3, label="Path")
        plt.scatter(
            [start_world[0]],
            [start_world[1]],
            s=120,
            edgecolors="black",
            linewidths=1.5,
            marker="o",
            label="Start",
        )
        plt.scatter(
            [goal_world[0]],
            [goal_world[1]],
            s=150,
            edgecolors="black",
            linewidths=1.5,
            marker="^",
            label="Goal",
        )
        plt.title(title)
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.axis("equal")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
    except Exception as e:
        pass


def get_world_bounds_from_simulation() -> Tuple[float, float, float, float]:
    session = get_coppelia_session()
    sim = session.simulation

    session.start()
    session.wait_until_running()

    scene_floor_handle = sim.getObject("/scene/floor")
    max_x = sim.getObjectFloatParameter(
        scene_floor_handle, sim.objfloatparam_objbbox_max_x
    )[1]
    min_x = sim.getObjectFloatParameter(
        scene_floor_handle, sim.objfloatparam_objbbox_min_x
    )[1]
    max_y = sim.getObjectFloatParameter(
        scene_floor_handle, sim.objfloatparam_objbbox_max_y
    )[1]
    min_y = sim.getObjectFloatParameter(
        scene_floor_handle, sim.objfloatparam_objbbox_min_y
    )[1]

    session.stop()
    session.wait_until_stopped()

    bounds = (min_x, max_x, min_y, max_y)

    return bounds


def get_robot_and_goal_positions(
    robot_type: str,
) -> Tuple[Tuple[float, float], Tuple[float, float]]:
    session = get_coppelia_session()
    sim = session.simulation

    session.start()
    session.wait_until_running()

    if robot_type == "differential":
        robot_handle = sim.getObject("/ePuck")
    else:
        robot_handle = sim.getObject("/OmniPlatform")

    goal_handle = sim.getObject("/goal")

    robot_pose = session.get_pose2d_world(robot_handle)
    goal_pose = session.get_pose2d_world(goal_handle)

    start_world = (robot_pose.x, robot_pose.y)
    goal_world = (goal_pose.x, goal_pose.y)

    session.stop()
    session.wait_until_stopped()

    return start_world, goal_world


def save_path_result(
    path_world: List[Tuple[float, float]],
    scene: str,
    robot_type: str,
    algorithm: str,
    output_dir: str,
) -> str:
    output_path = resolve_output_path(output_dir)
    ensure_directory_exists(output_path)

    out_file = output_path / f"{scene}_{robot_type}_{algorithm}_path.json"
    with open(out_file, "w") as f:
        json.dump(path_world, f, indent=2)

    return str(out_file)


def get_saved_path_file(
    scene: str, robot_type: str, algorithm: str, output_dir: str
) -> Path:
    output_path = resolve_output_path(output_dir)
    return output_path / f"{scene}_{robot_type}_{algorithm}_path.json"


def load_path_result(
    scene: str,
    robot_type: str,
    algorithm: str,
    output_dir: str,
) -> Optional[List[Tuple[float, float]]]:
    path_file = get_saved_path_file(scene, robot_type, algorithm, output_dir)
    if not path_file.exists():
        return None
    with open(path_file, "r") as f:
        data = json.load(f)
    path: List[Tuple[float, float]] = [(float(x), float(y)) for x, y in data]
    return path


def load_scene(scene_path: str) -> None:
    session = get_coppelia_session()
    sim = session.simulation

    session.start()
    session.wait_until_running()

    if sim.getSimulationState() != sim.simulation_stopped:
        sim.stopSimulation()
        while sim.getSimulationState() != sim.simulation_stopped:
            session.step()

    sim.loadScene(scene_path)

    session.stop()
    session.wait_until_stopped()
