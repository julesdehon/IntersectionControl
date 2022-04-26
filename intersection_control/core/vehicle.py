from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Tuple, Optional

from intersection_control.core import Environment

'''
Vehicles
-------------
Either communicate with each-other or the intersection manager in order to figure out how to proceed through the
intersection
'''


class Vehicle(ABC):
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

    def approaching(self) -> Optional[str]:
        return self.environment.vehicles.approaching(self.vehicle_id)

    def departing(self) -> Optional[str]:
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

    def get_position(self) -> Tuple[float, float]:
        return self.environment.vehicles.get_position(self.vehicle_id)

    def set_desired_speed(self, to: float):
        return self.environment.vehicles.set_desired_speed(self.vehicle_id, to)
