from utils.session import get_coppelia_session

from robots.differential import (
    controller as differential_controller,
    navigator as differential_navigator,
    actuator as differential_actuator,
    kinematics as differential_kinematics,
)
from robots.holonomic import (
    controller as holonomic_controller,
    navigator as holonomic_navigator,
    actuator as holonomic_actuator,
    kinematics as holonomic_kinematics,
)


def get_robot_components(robot_type: str):
    session = get_coppelia_session()

    if robot_type == "differential":
        left_motor, right_motor = differential_kinematics.motor_paths()
        return {
            "controller": differential_controller.DifferentialController(),
            "navigator": differential_navigator.DifferentialNavigator(),
            "actuator": differential_actuator.DifferentialActuator(
                session, left_motor, right_motor
            ),
            "body_path": differential_kinematics.body_path(),
            "kinematics": differential_kinematics.get_params(),
        }

    elif robot_type == "holonomic":
        left_top, right_top, left_bottom, right_bottom = (
            holonomic_kinematics.wheel_paths()
        )
        return {
            "controller": holonomic_controller.HolonomicController(),
            "navigator": holonomic_navigator.HolonomicNavigator(),
            "actuator": holonomic_actuator.HolonomicActuator(
                session, left_top, right_top, left_bottom, right_bottom
            ),
            "body_path": holonomic_kinematics.body_path(),
            "kinematics": holonomic_kinematics.get_params(),
        }

    else:
        raise ValueError(f"Unknown robot type: {robot_type}")
