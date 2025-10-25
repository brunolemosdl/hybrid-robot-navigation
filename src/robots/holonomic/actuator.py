from robots.base.actuator import BaseActuator
from services.coppelia import CoppeliaSimSession


class HolonomicActuator(BaseActuator):
    def __init__(
        self,
        session: CoppeliaSimSession,
        left_top_path: str,
        right_top_path: str,
        left_bottom_path: str,
        right_bottom_path: str,
    ) -> None:
        super().__init__(session)
        self.left_top_path = left_top_path
        self.right_top_path = right_top_path
        self.left_bottom_path = left_bottom_path
        self.right_bottom_path = right_bottom_path

    def set_velocity(
        self,
        velocity_x: float,
        velocity_y: float,
        angular_velocity_z: float,
        wheel_radius: float,
        length_x: float,
        length_y: float,
        gear_ratio: float,
    ) -> None:
        self.session._ensure_ready()

        lt = self.session.get_object(self.left_top_path)
        rt = self.session.get_object(self.right_top_path)
        lb = self.session.get_object(self.left_bottom_path)
        rb = self.session.get_object(self.right_bottom_path)

        length_sum = length_x + length_y

        w1 = (-velocity_x + velocity_y - length_sum * angular_velocity_z) / wheel_radius
        w2 = (-velocity_x - velocity_y - length_sum * angular_velocity_z) / wheel_radius
        w3 = (velocity_x - velocity_y - length_sum * angular_velocity_z) / wheel_radius
        w4 = (velocity_x + velocity_y - length_sum * angular_velocity_z) / wheel_radius

        for handle, speed in zip(
            [lt, rt, lb, rb],
            [w1, w2, w3, w4],
        ):
            self.session.simulation.setJointTargetVelocity(
                handle, float(speed * gear_ratio)
            )

    def stop(self) -> None:
        self.session._ensure_ready()
        for path in [
            self.left_top_path,
            self.right_top_path,
            self.left_bottom_path,
            self.right_bottom_path,
        ]:
            joint = self.session.get_object(path)
            self.session.simulation.setJointTargetVelocity(joint, 0.0)
