from dataclasses import dataclass
from typing import Iterable, Tuple, Protocol

from robots.base.controller import BaseController


@dataclass
class NavigationConfig:
    waypoint_tolerance: float = 0.05
    dt_seconds: float = 0.05


class NavigatorProtocol(Protocol):
    def follow_path(
        self,
        path_world: Iterable[Tuple[float, float]],
        controller: "BaseController",
        *args,
        **kwargs,
    ) -> None: ...
