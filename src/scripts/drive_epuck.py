import sys
from utils import setup_logging, validate_arguments, print_configuration, ensure_directory_exists, get_coppelia_session
import argparse

def main():
    parser = argparse.ArgumentParser(
        description="Path Planning and Navigation - Mobile Robotics TP2"
    )
    parser.add_argument(
        "algorithm",
        choices=["rrt", "roadmap", "wavefront", "potential_fields", "extract_map"],
    )
    parser.add_argument("robot", choices=["differential", "holonomic"])
    parser.add_argument("scene", choices=["maze_1", "maze_2"])
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
    args = parser.parse_args()

    try:
        logger = setup_logging(verbose=args.verbose, log_to_file=True, log_dir="logs")

        validate_arguments(args)
        print_configuration(args, logger)
        ensure_directory_exists(args.output_dir)

        get_coppelia_session(args.host, args.port)
    except KeyboardInterrupt:
        logger.warning("Process interrupted by user.")
        sys.exit(1)
    except Exception as e:
        logger.error(f"An error occurred: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
