from __future__ import annotations

from abc import ABC, abstractmethod

from intersection_control.core.communication import CommunicativeAgent

'''
Intersection Manager
-------------
Communicates with Vehicles and coordinates their movements through the intersection
'''


class IntersectionManager(CommunicativeAgent, ABC):
    def __init__(self, env_interface: IMEnvironmentInterface):
        self.env_interface = env_interface

    @abstractmethod
    def step(self):
        pass


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
