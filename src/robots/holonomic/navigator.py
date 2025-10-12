import time
from typing import Iterable, Tuple

from robots.base.navigator import NavigationConfig
from robots.holonomic.kinematics import get_params, body_path, wheel_paths
from robots.holonomic.actuator import HolonomicActuator
from utils.session import get_coppelia_session


class HolonomicNavigator:
    def __init__(self, config: NavigationConfig | None = None) -> None:
        self.config = config or NavigationConfig()

    def follow_path(
        self,
        path_world: Iterable[Tuple[float, float]],
        controller,
    ) -> None:
        session = get_coppelia_session()
        waypoints = list(path_world)
        if not waypoints:
            return

        session.start()
        session.wait_until_running()

        wheel_radius, lx, ly, gear_ratio = get_params()
        left_top, right_top, left_bottom, right_bottom = wheel_paths()

        actuator = HolonomicActuator(
            session,
            left_top,
            right_top,
            left_bottom,
            right_bottom,
        )
        robot = session.get_object(body_path())

        target_index = 0
        while target_index < len(waypoints):
            pose = session.get_pose2d_world(robot)
            target_x, target_y = waypoints[target_index]

            control = controller.compute_control(
                (pose.x, pose.y, pose.theta),
                (target_x, target_y, 0.0),
            )

            actuator.set_velocity(
                control.linear_velocity_x,
                control.linear_velocity_y,
                control.angular_velocity_z,
                wheel_radius,
                lx,
                ly,
                gear_ratio,
            )

            dx, dy = target_x - pose.x, target_y - pose.y
            if (dx * dx + dy * dy) ** 0.5 <= self.config.waypoint_tolerance:
                target_index += 1

            time.sleep(self.config.dt_seconds)

        actuator.stop()
        session.stop()
        session.wait_until_stopped()
