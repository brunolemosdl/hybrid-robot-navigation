def get_params() -> tuple[float, float]:
    from .config import WHEEL_RADIUS, AXLE_LENGTH

    return WHEEL_RADIUS, AXLE_LENGTH


def motor_paths() -> tuple[str, str]:
    from .config import LEFT_MOTOR_PATH, RIGHT_MOTOR_PATH

    return LEFT_MOTOR_PATH, RIGHT_MOTOR_PATH


def body_path() -> str:
    from .config import ROBOT_BODY_PATH

    return ROBOT_BODY_PATH
