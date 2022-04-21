from __future__ import annotations

from abc import ABC, abstractmethod

from intersection_control.core.communication import CommunicativeAgent

'''
Vehicles
-------------
Either communicate with each-other or the intersection manager in order to figure out how to proceed through the
intersection
'''


class Vehicle(CommunicativeAgent, ABC):
    def __init__(self, env_interface: VehicleEnvironmentInterface):
        self.env_interface = env_interface

    @abstractmethod
    def step(self):
        pass


class VehicleEnvironmentInterface(ABC):
    @abstractmethod
    def approaching(self, communication_range: int) -> bool:
        pass

    @abstractmethod
    def departing(self) -> bool:
        pass

    @abstractmethod
    def in_intersection(self) -> bool:
        pass

    @abstractmethod
    def get_id(self) -> str:
        pass

    @abstractmethod
    def get_trajectory(self) -> str:
        pass

    @abstractmethod
    def get_length(self) -> float:
        pass

    @abstractmethod
    def get_width(self) -> float:
        pass

    @abstractmethod
    def get_driving_distance(self) -> float:
        pass

    @abstractmethod
    def get_velocity(self) -> float:
        pass

    @abstractmethod
    def set_desired_speed(self, to: float):
        pass

    @abstractmethod
    def get_current_time(self) -> float:
        pass

    @abstractmethod
    def get_speed_through_trajectory(self) -> float:
        pass
