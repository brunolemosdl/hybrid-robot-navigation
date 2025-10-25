import time
from typing import Iterable, Tuple

from utils.session import get_coppelia_session

from robots.base.navigator import NavigationConfig
from robots.differential.kinematics import get_params, body_path
from robots.differential.actuator import DifferentialActuator


class DifferentialNavigator:
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

        wheel_radius, axle_length = get_params()
        actuator = DifferentialActuator(
            session, "/ePuck/leftJoint", "/ePuck/rightJoint"
        )
        body = session.get_object(body_path())

        target_index = 0
        max_wheel_speed = 20.0

        while target_index < len(waypoints):
            pose = session.get_pose2d_world(body)
            target_x, target_y = waypoints[target_index]

            control = controller.compute_control(
                (pose.x, pose.y, pose.theta), (target_x, target_y, 0.0)
            )

            left_speed = (
                control.linear_velocity_x
                - 0.5 * axle_length * control.angular_velocity_z
            ) / wheel_radius
            right_speed = (
                control.linear_velocity_x
                + 0.5 * axle_length * control.angular_velocity_z
            ) / wheel_radius

            actuator.set_velocity(
                max(-max_wheel_speed, min(max_wheel_speed, left_speed)),
                max(-max_wheel_speed, min(max_wheel_speed, right_speed)),
            )

            dx, dy = target_x - pose.x, target_y - pose.y
            if dx * dx + dy * dy <= self.config.waypoint_tolerance**2:
                target_index += 1

            time.sleep(self.config.dt_seconds)

        actuator.stop()
        session.stop()
        session.wait_until_stopped()
