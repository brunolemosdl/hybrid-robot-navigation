from __future__ import annotations

from dataclasses import dataclass, asdict
from statistics import mean
from typing import Dict, Iterable, List, Optional, Tuple


@dataclass(frozen=True)
class EpisodeMetrics:
    """Stores the metrics collected for a single simulation episode."""

    reached_goal: bool
    collisions: int
    time_to_goal: Optional[float]
    path_length: Optional[float]
    min_distance: Optional[float]
    smoothness: Optional[float]

    def to_dict(self) -> Dict[str, Optional[float]]:
        """Expose the metrics as a serialisable dictionary."""
        return asdict(self)


class MetricsTracker:
    """Collects and aggregates metrics produced by multiple simulation episodes."""

    def __init__(self) -> None:
        self._episodes: List[EpisodeMetrics] = []

    def __len__(self) -> int:
        return len(self._episodes)

    def __iter__(self) -> Iterable[EpisodeMetrics]:
        return iter(self._episodes)

    def reset(self) -> None:
        """Drop all stored episodes."""
        self._episodes.clear()

    def record_episode(
        self,
        *,
        reached_goal: bool,
        collisions: int,
        time_to_goal: Optional[float],
        path_length: Optional[float],
        min_distance: Optional[float],
        smoothness: Optional[float],
    ) -> EpisodeMetrics:
        if collisions < 0:
            raise ValueError("Number of collisions cannot be negative.")

        metrics = EpisodeMetrics(
            reached_goal=reached_goal,
            collisions=collisions,
            time_to_goal=self._validate_non_negative(time_to_goal, "time_to_goal"),
            path_length=self._validate_non_negative(path_length, "path_length"),
            min_distance=self._validate_non_negative(min_distance, "min_distance"),
            smoothness=self._validate_non_negative(smoothness, "smoothness"),
        )
        self._episodes.append(metrics)
        return metrics

    @staticmethod
    def _validate_non_negative(
        value: Optional[float], field_name: str
    ) -> Optional[float]:
        if value is None:
            return None
        if value < 0:
            raise ValueError(f"{field_name} cannot be negative.")
        return value

    @property
    def episode_count(self) -> int:
        return len(self._episodes)

    @property
    def success_rate(self) -> float:
        """Fraction of episodes that reached the goal without collisions."""
        if not self._episodes:
            return 0.0
        successes = sum(
            1 for episode in self._episodes if episode.reached_goal and episode.collisions == 0
        )
        return successes / len(self._episodes)

    @property
    def collisions_history(self) -> Tuple[int, ...]:
        return tuple(episode.collisions for episode in self._episodes)

    @property
    def average_collisions(self) -> Optional[float]:
        if not self._episodes:
            return None
        return mean(self.collisions_history)

    @property
    def time_to_goal_history(self) -> Tuple[Optional[float], ...]:
        return tuple(episode.time_to_goal for episode in self._episodes)

    @property
    def average_time_to_goal(self) -> Optional[float]:
        return self._mean_of("time_to_goal")

    @property
    def path_length_history(self) -> Tuple[Optional[float], ...]:
        return tuple(episode.path_length for episode in self._episodes)

    @property
    def average_path_length(self) -> Optional[float]:
        return self._mean_of("path_length")

    @property
    def min_distance_history(self) -> Tuple[Optional[float], ...]:
        return tuple(episode.min_distance for episode in self._episodes)

    @property
    def average_min_distance(self) -> Optional[float]:
        return self._mean_of("min_distance")

    @property
    def smoothness_history(self) -> Tuple[Optional[float], ...]:
        return tuple(episode.smoothness for episode in self._episodes)

    @property
    def average_smoothness(self) -> Optional[float]:
        return self._mean_of("smoothness")

    def last_episode(self) -> Optional[EpisodeMetrics]:
        if not self._episodes:
            return None
        return self._episodes[-1]

    def to_dict(self) -> Dict[str, object]:
        """Return the stored metrics and aggregates as a dictionary."""
        return {
            "episode_count": self.episode_count,
            "success_rate": self.success_rate,
            "collisions_history": list(self.collisions_history),
            "average_collisions": self.average_collisions,
            "time_to_goal_history": list(self.time_to_goal_history),
            "average_time_to_goal": self.average_time_to_goal,
            "path_length_history": list(self.path_length_history),
            "average_path_length": self.average_path_length,
            "min_distance_history": list(self.min_distance_history),
            "average_min_distance": self.average_min_distance,
            "smoothness_history": list(self.smoothness_history),
            "average_smoothness": self.average_smoothness,
        }

    def _mean_of(self, attribute: str) -> Optional[float]:
        values = [
            value
            for value in (getattr(episode, attribute) for episode in self._episodes)
            if value is not None
        ]
        if not values:
            return None
        return mean(values)
