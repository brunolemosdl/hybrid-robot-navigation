def get_params() -> tuple[float, float, float, float]:
    from .config import WHEEL_RADIUS, LENGTH_X, LENGTH_Y, GEAR_RATIO

    return WHEEL_RADIUS, LENGTH_X, LENGTH_Y, GEAR_RATIO


def wheel_paths() -> tuple[str, str, str, str]:
    from .config import (
        LEFT_TOP_WHEEL_PATH,
        RIGHT_TOP_WHEEL_PATH,
        LEFT_BOTTOM_WHEEL_PATH,
        RIGHT_BOTTOM_WHEEL_PATH,
    )

    return (
        LEFT_TOP_WHEEL_PATH,
        RIGHT_TOP_WHEEL_PATH,
        LEFT_BOTTOM_WHEEL_PATH,
        RIGHT_BOTTOM_WHEEL_PATH,
    )


def body_path() -> str:
    from .config import ROBOT_BODY_PATH

    return ROBOT_BODY_PATH
