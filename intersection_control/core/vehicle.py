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
        """Returns the id of the intersection the vehicle is approaching, or None
        if it is not approaching an intersection

        :return: Either the ID of the intersection being approached, or None
        """
        return self.environment.vehicles.approaching(self.vehicle_id)

    def departing(self) -> Optional[str]:
        """Returns the id of the intersection the vehicle is departing, or None
        if it is not departing an intersection

        :return: Either the ID of the intersection being departed, or None
        """
        return self.environment.vehicles.departing(self.vehicle_id)

    def in_intersection(self) -> bool:
        """Returns True iff the vehicle is currently inside an intersection

        :return: True iff the vehicle is currently inside an intersection
        """
        return self.environment.vehicles.in_intersection(self.vehicle_id)

    def get_trajectory(self) -> str:
        """Returns the ID of the trajectory the vehicle is planning to take through the
        next intersection

        TODO: Rethink this... See the TODO comment in the SUMOIntersectionHandler
            class

        :return: Trajectory ID
        """
        return self.environment.vehicles.get_trajectory(self.vehicle_id)

    def get_length(self) -> float:
        """Returns the length of the vehicle

        :return: Length of the vehicle in metres
        """
        return self.environment.vehicles.get_length(self.vehicle_id)

    def get_width(self) -> float:
        """Returns the width of the given vehicle

        :return: Width of the vehicle in metres
        """
        return self.environment.vehicles.get_width(self.vehicle_id)

    def get_driving_distance(self) -> float:
        """Returns the driving distance of the vehicle to the end of the current edge/road

        TODO: This is not very clean - seems quite particular to the QBIM algorithm...
            Worth thinking about whether there is a better, more general way of providing
            the information required to figure this out.

        :return: The driving distance in metres
        """
        return self.environment.vehicles.get_driving_distance(self.vehicle_id)

    def get_speed(self) -> float:
        """Returns the current speed of the vehicle

        :return: The speed in metres/second
        """
        return self.environment.vehicles.get_speed(self.vehicle_id)

    def get_position(self) -> Tuple[float, float]:
        """Returns the current position of the vehicle

        :return: The (x,y) position of the vehicle
        """
        return self.environment.vehicles.get_position(self.vehicle_id)

    def get_direction(self) -> float:
        """Returns the direction the given vehicle is currently facing in radians

        The positive horizontal axis is taken to have a direction of 0, and the
        angle increases as the vehicle rotates clockwise like so:

              3pi/2
               ^
               |
        pi <---+---> 0
               |
               v
              pi/2

        :return: The direction of the vehicle in radians
        """
        return self.environment.vehicles.get_direction(self.vehicle_id)

    def set_desired_speed(self, to: float):
        """Sets the desired speed of the given vehicle

        :param to: The speed (in m/s) you would like the desired speed to be
            set to. If set to -1, the vehicle's speed will be controlled by
            the environment.
        """
        self.environment.vehicles.set_desired_speed(self.vehicle_id, to)

    def get_id(self) -> str:
        """Return the vehicle id

        :return: vehicle id
        """
        return self.vehicle_id

    def get_speed_limit(self) -> float:
        """Returns the maximum speed the vehicle should be travelling at

        This will usually be determined by the speed limit of the road the vehicle
        is currently on
        """
        return self.environment.vehicles.get_speed_limit(self.vehicle_id)
