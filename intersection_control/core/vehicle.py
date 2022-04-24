from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from intersection_control.core.communication import CommunicativeAgent
from intersection_control.core import Environment

'''
Vehicles
-------------
Either communicate with each-other or the intersection manager in order to figure out how to proceed through the
intersection
'''


class Vehicle(CommunicativeAgent, ABC):
    def __init__(self, vehicle_id: str, environment: Environment):
        self.vehicle_id = vehicle_id
        self.environment = environment

    @abstractmethod
    def step(self):
        """Called repeatedly after some time has elapsed

        Can assume that the state of the environment has changed since step
        was last called. This is where the main behaviour of the vehicle
        should be implemented"""
        pass

    def approaching(self) -> str:
        return self.environment.vehicles.approaching(self.vehicle_id)

    def departing(self) -> str:
        return self.environment.vehicles.departing(self.vehicle_id)

    def in_intersection(self) -> bool:
        return self.environment.vehicles.in_intersection(self.vehicle_id)

    def get_trajectory(self) -> str:
        return self.environment.vehicles.get_trajectory(self.vehicle_id)

    def get_length(self) -> float:
        return self.environment.vehicles.get_length(self.vehicle_id)

    def get_width(self) -> float:
        return self.environment.vehicles.get_width(self.vehicle_id)

    def get_driving_distance(self) -> float:
        return self.environment.vehicles.get_driving_distance(self.vehicle_id)

    def get_speed(self) -> float:
        return self.environment.vehicles.get_speed(self.vehicle_id)

    def set_desired_speed(self, to: float):
        return self.environment.vehicles.set_desired_speed(self.vehicle_id, to)


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
