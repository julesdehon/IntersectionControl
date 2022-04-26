from abc import ABC, abstractmethod

from typing import List, Tuple, Optional


class VehicleHandler(ABC):
    @abstractmethod
    def approaching(self, vehicle_id: str) -> Optional[str]:
        """Returns the id of the intersection the given vehicle is approaching, or None
        if it is not approaching an intersection"""
        raise NotImplementedError

    @abstractmethod
    def departing(self, vehicle_id: str) -> Optional[str]:
        """Returns the id of the intersection the given vehicle is departing, or None
        if it is not departing an intersection"""
        raise NotImplementedError

    @abstractmethod
    def in_intersection(self, vehicle_id: str) -> bool:
        raise NotImplementedError

    @abstractmethod
    def get_ids(self) -> List[str]:
        raise NotImplementedError

    @abstractmethod
    def get_trajectory(self, vehicle_id: str) -> str:
        raise NotImplementedError

    @abstractmethod
    def get_length(self, vehicle_id: str) -> float:
        raise NotImplementedError

    @abstractmethod
    def get_width(self, vehicle_id: str) -> float:
        raise NotImplementedError

    @abstractmethod
    def get_driving_distance(self, vehicle_id: str) -> float:
        """Returns the driving distance of the vehicle to the end of the current edge/road"""
        raise NotImplementedError

    @abstractmethod
    def get_speed(self, vehicle_id: str) -> float:
        raise NotImplementedError

    @abstractmethod
    def set_desired_speed(self, vehicle_id: str, to: float):
        """Sets the desired speed of the given vehicle

        Special case when *to* is set to -1: The vehicle's speed will return
        to a "default", controlled by the environment - e.g. in SUMO this means
        the vehicle's speed will be controlled by the default car-following
        models

        :param str vehicle_id: The ID of the vehicle you would like to change
            the desired speed of
        :param to: The speed (in m/s) you would like the desired speed to be
            set to. If set to -1, the vehicle's speed will be controlled by
            the environment.
        """
        raise NotImplementedError

    @abstractmethod
    def get_position(self, vehicle_id) -> Tuple[float, float]:
        raise NotImplementedError
