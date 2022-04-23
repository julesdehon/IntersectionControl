from abc import ABC, abstractmethod

from .intersectionhandler import IntersectionHandler
from .vehiclehandler import VehicleHandler


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
        """Returns the current time in the environment in seconds"""
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
