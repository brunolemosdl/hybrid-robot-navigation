from robots.base.actuator import BaseActuator
from services.coppelia import CoppeliaSimSession


class DifferentialActuator(BaseActuator):
    def __init__(
        self, session: CoppeliaSimSession, left_motor_path: str, right_motor_path: str
    ) -> None:
        super().__init__(session)
        self.left_motor_path = left_motor_path
        self.right_motor_path = right_motor_path

    def set_velocity(self, left_speed: float, right_speed: float) -> None:
        self.session._ensure_ready()
        left_motor = self.session.get_object(self.left_motor_path)
        right_motor = self.session.get_object(self.right_motor_path)
        self.session.simulation.setJointTargetVelocity(left_motor, left_speed)
        self.session.simulation.setJointTargetVelocity(right_motor, right_speed)

    def stop(self) -> None:
        self.set_velocity(0.0, 0.0)
