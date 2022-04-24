from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Dict
from intersection_control.core.communication import CommunicativeAgent
from intersection_control.core import Environment
from intersection_control.core.environment import Trajectory

'''
Intersection Manager
-------------
Communicates with Vehicles and coordinates their movements through the intersection
'''


class IntersectionManager(CommunicativeAgent, ABC):
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

    def get_trajectories(self) -> Dict[str, Trajectory]:
        """Return all the possible trajectories through the intersection

        :return: A dictionary mapping trajectory ids to trajectories
        """
        return self.environment.intersections.get_trajectories(self.intersection_id)


class IMEnvironmentInterface(ABC):
    @abstractmethod
    def get_current_time(self):
        pass

    @abstractmethod
    def get_width(self):
        pass

    @abstractmethod
    def get_height(self):
        pass

    @abstractmethod
    def get_trajectories(self):
        pass
