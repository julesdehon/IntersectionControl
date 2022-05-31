from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict, Tuple, Set
from intersection_control.core import Environment
from intersection_control.core.environment import Trajectory

'''
Intersection Manager
-------------
Communicates with Vehicles and coordinates their movements through the intersection
'''


class IntersectionManager(ABC):
    def __init__(self, intersection_id: str, environment: Environment):
        self.environment = environment
        self.intersection_id = intersection_id

    @abstractmethod
    def step(self):
        """Called repeatedly after some time has elapsed

        Can assume that the state of the environment has changed since step
        was last called. This is where the main behaviour of the intersection
        manager should be implemented"""
        raise NotImplementedError

    def get_width(self) -> float:
        """Return the width of the intersection

        :return: The width of the intersection in metres
        """
        return self.environment.intersections.get_width(self.intersection_id)

    def get_height(self) -> float:
        """Return the height of the intersection

        :return: The height of the intersection in metres
        """
        return self.environment.intersections.get_height(self.intersection_id)

    def get_position(self) -> Tuple[float, float]:
        """Return the position of the intersection

        :return: (x,y) the position of the intersection
        """
        return self.environment.intersections.get_position(self.intersection_id)

    def get_trajectories(self) -> Dict[str, Trajectory]:
        """Return all the possible trajectories through the intersection

        :return: A dictionary mapping trajectory ids to trajectories
        """
        return self.environment.intersections.get_trajectories(self.intersection_id)

    def get_id(self) -> str:
        """Returns the intersection's id

        :return: intersection id
        """
        return self.intersection_id

    def set_traffic_light_phase(self, phase: Tuple[Set[str], Set[str], Set[str]]):
        """Set the traffic lights to the phase described by phase

        :param Tuple[Set[str], Set[str], Set[str]] phase: A tuple of length 3: (Green, Yellow, Red)  where each of
            Green, Yellow and Red are sets of trajectory IDs such that all the trajectories in Red will be shown a red
            light, and the same for Yellow and Green
        """
        self.environment.intersections.set_traffic_light_phase(self.intersection_id, phase)
