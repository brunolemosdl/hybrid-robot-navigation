import os
import sys
import signal
import atexit
import argparse
import socket

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from algorithms import (
    RRTPlanner,
    RoadmapPlanner,
    WavefrontPlanner,
    PotentialFieldsPlanner,
)

from robots import get_robot_components

from utils.logger import setup_logging
from utils.session import get_coppelia_session, reset_coppelia_session
from utils.map import (
    extract_map_from_simulation,
    visualize_map_quick,
    visualize_map_with_path,
    get_world_bounds_from_simulation,
    get_robot_and_goal_positions,
    save_path_result,
    load_path_result,
    get_saved_path_file,
    load_scene,
)
from utils.paths import (
    resolve_scene_path,
    ensure_directory_exists,
    validate_scene_file,
    get_relative_path_for_display,
)

GLOBAL_MAP = {
    "rrt": RRTPlanner,
    "roadmap": RoadmapPlanner,
    "wavefront": WavefrontPlanner,
    "potential_fields": PotentialFieldsPlanner,
}

VALID_SCENES = ["scene_1", "scene_2", "scene_3", "scene_4"]

def validate_arguments(args):
    valid_algorithms = [
        "rrt",
        "roadmap",
        "wavefront",
        "potential_fields",
        "extract_map",
    ]
    valid_robots = ["differential", "holonomic"]

    if args.algorithm not in valid_algorithms:
        raise ValueError(f"Invalid algorithm '{args.algorithm}'")

    if args.robot not in valid_robots:
        raise ValueError(f"Invalid robot type '{args.robot}'")

    if args.scene not in VALID_SCENES:
        raise ValueError(f"Invalid scene '{args.scene}'")

    if not validate_scene_file(args.scene_path, args.scene):
        scene_path = resolve_scene_path(args.scene_path, args.scene)
        raise FileNotFoundError(f"Scene file not found: {scene_path}")

    return True


def print_configuration(args, logger):
    scene_file = resolve_scene_path(args.scene_path, args.scene)
    display_path = get_relative_path_for_display(scene_file)

    logger.info("=" * 60)
    logger.info("PATH PLANNING AND NAVIGATION")
    logger.info("=" * 60)
    logger.info(f"Global Planner: {args.global_planner.upper()}")
    logger.info(f"Local Planner : {args.local_planner.upper()}")
    logger.info(f"Robot Type    : {args.robot.upper()}")
    logger.info(f"Scene         : {args.scene.upper()}")
    logger.info(f"Scene Path    : {display_path}")
    logger.info(f"Coppelia Host : {args.host}")
    logger.info(f"Coppelia Port : {args.port}")
    logger.info(f"Visualization : {'ON' if args.visualize else 'OFF'}")
    logger.info(f"Verbose Mode  : {'ON' if args.verbose else 'OFF'}")
    logger.info("=" * 60)


def check_coppelia_connection(host: str, port: int, timeout: float = 2.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def run_planner(args, PlannerClass, logger):
    logger.info(f"Running {PlannerClass.__name__}...")

    if not check_coppelia_connection(args.host, args.port):
        logger.error(
            f"Could not connect to CoppeliaSim at {args.host}:{args.port}. "
            "Make sure CoppeliaSim is running and ZMQ Remote API Server is started "
            "(Modules → Connectivity → ZMQ Remote API Server)."
        )
        sys.exit(1)

    scene_file = resolve_scene_path(args.scene_path, args.scene)
    display_path = get_relative_path_for_display(scene_file)

    logger.info(f"Loading scene: {display_path}")
    logger.debug(f"Full scene path: {scene_file}")

    load_scene(str(scene_file))

    logger.info("Extracting map from simulation...")

    world_bounds = get_world_bounds_from_simulation()
    logger.debug(f"World bounds: {world_bounds}")

    start_world, goal_world = get_robot_and_goal_positions(args.robot)
    logger.debug(f"Start position: {start_world}")
    logger.debug(f"Goal position: {goal_world}")

    binary_map = extract_map_from_simulation(
        sensor_path="/scene/floor/sensor",
        clean_map=True,
        save_map=False,
        scene=args.scene,
        robot_type=args.robot,
        output_dir=args.output_dir,
        selected_robot=args.robot,
    )

    if binary_map is None:
        logger.error("Map extraction failed")
        return {"success": False, "message": "Map extraction failed"}

    resolution = binary_map.shape[::-1]
    logger.debug(f"Map resolution: {resolution}")
    logger.info(f"Map extracted successfully (size: {binary_map.shape})")

    if args.visualize:
        logger.info("Visualizing extracted map...")
        visualize_map_quick(binary_map, f"Extracted Map - {args.scene}", world_bounds)

    logger.info(f"Starting path planning with {PlannerClass.__name__}...")
    planner = PlannerClass()
    path_world = planner.plan(
        binary_map, start_world, goal_world, world_bounds, resolution
    )

    if not path_world:
        logger.warning("No path found by planner")
        return {"success": False, "message": "No path found"}

    logger.info(f"Path found with {len(path_world)} waypoints")
    logger.debug(f"First waypoint: {path_world[0]}")
    logger.debug(f"Last waypoint: {path_world[-1]}")

    if args.visualize:
        logger.info("Visualizing path...")
        visualize_map_with_path(
            binary_map=binary_map,
            path_world=path_world,
            world_bounds=world_bounds,
            start_world=start_world,
            goal_world=goal_world,
            title=f"Path - {args.global_planner} - {args.scene}",
        )

    out_file = save_path_result(
        path_world, args.scene, args.robot, args.global_planner, args.output_dir
    )

    logger.info(f"Path saved to {out_file}")

    return {
        "success": True,
        "path": path_world,
        "world_bounds": world_bounds,
        "start_world": start_world,
        "goal_world": goal_world,
    }


def run_navigation(args, path_world, logger):
    logger.info("Starting robot navigation...")

    components = get_robot_components(
        args.robot, getattr(args, "local_planner", "none")
    )
    controller = components["controller"]
    navigator = components["navigator"]
    body_path = components["body_path"]
    kinematics = components["kinematics"]

    logger.debug(f"Using controller: {controller.__class__.__name__}")
    logger.debug(f"Using navigator: {navigator.__class__.__name__}")
    logger.debug(f"Body path: {body_path}")
    logger.debug(f"Kinematics parameters: {kinematics}")

    navigator.follow_path(path_world, controller)

    logger.info("Navigation completed!")


def main():
    parser = argparse.ArgumentParser(
        description="Reinforcement Learning for Robot Path Planning and Navigation with CoppeliaSim"
    )

    # Mantido por compatibilidade com chamadas antigas
    parser.add_argument(
        "algorithm",
        choices=["rrt", "roadmap", "wavefront", "potential_fields", "extract_map"],
    )

    parser.add_argument("robot", choices=["differential", "holonomic"])
    parser.add_argument("scene", choices=VALID_SCENES)
    parser.add_argument("--scene-path", default="./scenes")
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--output-dir", default="./results")
    parser.add_argument("--navigate-only", action="store_true")
    parser.add_argument(
        "--host", default="localhost", help="CoppeliaSim ZMQ host (default: localhost)"
    )
    parser.add_argument(
        "--port", type=int, default=23000, help="CoppeliaSim ZMQ port (default: 23000)"
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Enable verbose logging with detailed debug information (default: False)",
    )

    # Novos seletores
    parser.add_argument(
        "--global",
        dest="global_planner",
        choices=["rrt", "roadmap", "wavefront", "potential_fields"],
        default="wavefront",
    )
    parser.add_argument(
        "--local", dest="local_planner", choices=["dwa", "none"], default="dwa"
    )

    args = parser.parse_args()

    try:
        logger = setup_logging(verbose=args.verbose, log_to_file=True, log_dir="logs")

        validate_arguments(args)
        print_configuration(args, logger)
        ensure_directory_exists(args.output_dir)

        get_coppelia_session(args.host, args.port)

        def _cleanup():
            logger.debug("Cleaning up Coppelia session before exit...")
            try:
                reset_coppelia_session()
            except Exception:
                logger.exception("Error while resetting Coppelia session")

        atexit.register(_cleanup)

        def _signal_handler(signum=None, frame=None):
            try:
                _cleanup()
            finally:
                sys.exit(1)

        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            try:
                signal.signal(sig, _signal_handler)
            except Exception:
                pass

        if args.algorithm == "extract_map":
            logger.info("Running map extraction only...")

            if not check_coppelia_connection(args.host, args.port):
                logger.error(
                    f"Could not connect to CoppeliaSim at {args.host}:{args.port}. "
                    "Make sure CoppeliaSim is running and ZMQ Remote API Server is started."
                )
                sys.exit(1)

            scene_file = resolve_scene_path(args.scene_path, args.scene)
            display_path = get_relative_path_for_display(scene_file)
            logger.info(f"Loading scene: {display_path}")
            logger.debug(f"Full scene path: {scene_file}")

            load_scene(str(scene_file))
            world_bounds = get_world_bounds_from_simulation()
            logger.debug(f"World bounds: {world_bounds}")

            binary_map = extract_map_from_simulation(
                sensor_path="/scene/floor/sensor",
                clean_map=True,
                save_map=True,
                scene=args.scene,
                robot_type=args.robot,
                output_dir=args.output_dir,
                selected_robot=args.robot,
            )

            if binary_map is None:
                logger.error("Map extraction failed")
                sys.exit(1)

            logger.info(f"Map extracted successfully (size: {binary_map.shape})")

            if args.visualize:
                visualize_map_quick(
                    binary_map, f"Extracted Map - {args.scene}", world_bounds
                )

            logger.info("Algorithm 'extract_map' completed successfully!")
            sys.exit(0)

        if args.navigate_only:
            logger.info("Loading saved path for navigation...")
            saved_path = load_path_result(
                args.scene, args.robot, args.global_planner, args.output_dir
            )

            if not saved_path:
                path_file = get_saved_path_file(
                    args.scene, args.robot, args.global_planner, args.output_dir
                )
                logger.error(
                    f"No saved path found at {get_relative_path_for_display(path_file)}. Run planning first."
                )
                sys.exit(1)

            logger.info(f"Found saved path with {len(saved_path)} waypoints")
            run_navigation(args, saved_path, logger)
            sys.exit(0)

        GlobalPlannerClass = GLOBAL_MAP[args.global_planner]
        result = run_planner(args, GlobalPlannerClass, logger)

        if not result.get("success") or not result.get("path"):
            logger.error(
                f"Global planner '{args.global_planner}' failed: {result.get('message')}"
            )
            sys.exit(1)

        path = result["path"]

        logger.info(
            f"Starting navigation with local planner: {args.local_planner.upper()}"
        )
        run_navigation(args, path, logger)
        sys.exit(0)

    except KeyboardInterrupt:
        logger.warning("Process interrupted by user")
        sys.exit(1)
    except Exception as e:
        logger.exception(f"Unexpected error occurred: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
