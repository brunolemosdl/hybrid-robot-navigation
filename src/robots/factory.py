from utils.session import get_coppelia_session
from algorithms.dwa import DWAController, DWAConfig

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


def get_robot_components(robot_type: str, local_planner: str = "none"):
    """
    Monta os componentes do robô e permite escolher um planner local (ex.: 'dwa').
    - robot_type: 'differential' | 'holonomic'
    - local_planner: 'none' | 'dwa'
    """
    session = get_coppelia_session()

    if robot_type == "differential":
        left_motor, right_motor = differential_kinematics.motor_paths()

        controller = differential_controller.DifferentialController()

        if local_planner and local_planner.lower() == "dwa":
            if DWAController is None:
                raise ImportError(
                    "DWAController não encontrado. Verifique o módulo controllers/dwa_controller.py"
                )

            dwa_cfg = DWAConfig() if DWAConfig else None
            controller = DWAController(
                config=dwa_cfg,
                obstacle_provider=None,
            )

        return {
            "controller": controller,
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

        if local_planner and local_planner.lower() == "dwa":
            raise NotImplementedError(
                "DWA para robô holonômico não está implementado. "
                "Adapte o controlador para (vx, vy, w) ou use 'none'."
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
