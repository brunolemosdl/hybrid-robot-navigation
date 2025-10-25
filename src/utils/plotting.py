import numpy as np
import matplotlib.pyplot as plt

from typing import Dict, Tuple, List

from .conversions import pixel_to_world


def plot_map_with_paths(
    binary_image: np.ndarray,
    positions_world: Dict[str, Tuple[float, float]],
    paths_pixels: Dict[str, List[Tuple[int, int]]],
    min_x: float,
    max_x: float,
    min_y: float,
    max_y: float,
    width: int,
    height: int,
):
    plt.figure(figsize=(12, 10))
    plt.imshow(
        binary_image,
        extent=[min_x, max_x, min_y, max_y],
        cmap="gray",
        alpha=0.7,
        origin="upper",
    )

    colors = {"ePuck": "red", "OmniPlatform": "blue", "Goal": "green"}
    markers = {"ePuck": "o", "OmniPlatform": "s", "Goal": "^"}

    for robot_name, path in paths_pixels.items():
        if path:
            world_path = [
                pixel_to_world(p, min_x, max_x, min_y, max_y, width, height)
                for p in path
            ]
            plt.plot(
                [p[0] for p in world_path],
                [p[1] for p in world_path],
                color=colors.get(robot_name, "black"),
                linewidth=2,
                alpha=0.8,
                label=f"{robot_name} path",
            )

    for name, (wx, wy) in positions_world.items():
        plt.scatter(
            wx,
            wy,
            c=colors.get(name, "black"),
            marker=markers.get(name, "x"),
            s=120,
            edgecolors="black",
            linewidths=1.5,
            label=name,
            zorder=5,
        )

    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Planned Path")
    plt.legend()
    plt.axis("equal")
    plt.grid(True, alpha=0.3)
    plt.show()
