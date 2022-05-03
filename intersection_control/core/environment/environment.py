from abc import ABC, abstractmethod
from typing import List
from .intersection_handler import IntersectionHandler
from .vehicle_handler import VehicleHandler


class Environment(ABC):
    @property
    @abstractmethod
    def intersections(self) -> IntersectionHandler:
        """Any environment interactions related to intersections should be
        performed through the intersection handler, accessible through this
        intersections property"""
        raise NotImplementedError

    @property
    @abstractmethod
    def vehicles(self) -> VehicleHandler:
        """Any environment interactions related to vehicles should be performed
        through the vehicle handler, accessible through this vehicles
        property"""
        raise NotImplementedError

    @abstractmethod
    def get_current_time(self) -> float:
        """Returns the current time in the environment in seconds

        :return: The current time in seconds
        """
        raise NotImplementedError

    @abstractmethod
    def step(self):
        """Performs a single step in the environment

        In a simulated environment, this should advance the simulation by a
        single time step, and in a real-time environment, this could either
        introduce a time delay - in order to tune the sampling frequency,
        or simply do nothing
        """
        raise NotImplementedError

    @abstractmethod
    def get_removed_vehicles(self) -> List[str]:
        """Returns a list of the vehicles removed from the environment in
        last time step

        In a simulation environment, this may be, for example, because the
        vehicle has completed its trip.

        :return: List of vehicle IDs removed from the environment
        """
        raise NotImplementedError

    @abstractmethod
    def get_added_vehicles(self) -> List[str]:
        """Returns a list of the vehicles added to the environment in
        the last time step

        In a simulation environment, a vehicle demand generator may be
        adding vehicles to the environment at certain entry points. This
        method allows the user know about these new vehicles entering
        the environment

        :return: List of vehicle IDs added to the environment
        """
        raise NotImplementedError

    @abstractmethod
    def clear(self):
        """Removes all vehicles from the environment"""
        raise NotImplementedError
