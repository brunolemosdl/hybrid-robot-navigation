from .paths import validate_scene_file, resolve_scene_path, get_relative_path_for_display

def validate_arguments(args):
    valid_algorithms = [
        "rrt",
        "roadmap",
        "wavefront",
        "potential_fields",
        "extract_map",
    ]
    valid_robots = ["differential", "holonomic"]
    valid_scenes = ["maze_1", "maze_2"]

    if args.algorithm not in valid_algorithms:
        raise ValueError(f"Invalid algorithm '{args.algorithm}'")

    if args.robot not in valid_robots:
        raise ValueError(f"Invalid robot type '{args.robot}'")

    if args.scene not in valid_scenes:
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
    logger.info(f"Algorithm     : {args.algorithm.upper()}")
    logger.info(f"Robot Type    : {args.robot.upper()}")
    logger.info(f"Scene         : {args.scene.upper()}")
    logger.info(f"Scene Path    : {display_path}")
    logger.info(f"Coppelia Host : {args.host}")
    logger.info(f"Coppelia Port : {args.port}")
    logger.info(f"Visualization : {'ON' if args.visualize else 'OFF'}")
    logger.info(f"Verbose Mode  : {'ON' if args.verbose else 'OFF'}")
    logger.info("=" * 60)