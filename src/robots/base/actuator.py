from abc import ABC, abstractmethod
from typing import Protocol

from services.coppelia import CoppeliaSimSession


class BaseActuator(ABC):
    def __init__(self, session: CoppeliaSimSession) -> None:
        self.session = session

    @abstractmethod
    def set_velocity(self, *args, **kwargs) -> None:
        pass

    @abstractmethod
    def stop(self) -> None:
        pass


class ActuatorProtocol(Protocol):
    def set_velocity(self, *args, **kwargs) -> None: ...
    def stop(self) -> None: ...
