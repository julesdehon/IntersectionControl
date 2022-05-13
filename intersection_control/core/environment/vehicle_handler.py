from abc import ABC, abstractmethod

from typing import List, Tuple, Optional


class VehicleHandler(ABC):
    @abstractmethod
    def approaching(self, vehicle_id: str) -> Optional[str]:
        """Returns the id of the intersection the given vehicle is approaching, or None
        if it is not approaching an intersection

        :param: str vehicle_id: The ID of the vehicle we are interested in
        :return: Either the ID of the intersection being approached, or None
        """
        raise NotImplementedError

    @abstractmethod
    def departing(self, vehicle_id: str) -> Optional[str]:
        """Returns the id of the intersection the given vehicle is departing, or None
        if it is not departing an intersection

        :param: str vehicle_id: The ID of the vehicle we are interested in
        :return: Either the ID of the intersection being departed, or None
        """
        raise NotImplementedError

    @abstractmethod
    def in_intersection(self, vehicle_id: str) -> bool:
        """Returns True iff the given vehicle is currently inside an intersection

        :param str vehicle_id: The ID of the vehicle we are interested in
        :return: True iff the given vehicle is currently inside an intersection
        """
        raise NotImplementedError

    @abstractmethod
    def get_ids(self) -> List[str]:
        """Returns a list of all vehicle IDs currently in the environment

        :return: List of vehicle IDs
        """
        raise NotImplementedError

    @abstractmethod
    def get_trajectory(self, vehicle_id: str) -> str:
        """Returns the ID of the trajectory the vehicle is planning to take through the
        next intersection

        :param str vehicle_id: The ID of the vehicle we are interested in
        :return: Trajectory ID
        """
        raise NotImplementedError

    @abstractmethod
    def get_length(self, vehicle_id: str) -> float:
        """Returns the length of the given vehicle

        :param str vehicle_id: The ID of the vehicle we are interested in
        :return: Length of the vehicle in metres
        """
        raise NotImplementedError

    @abstractmethod
    def get_width(self, vehicle_id: str) -> float:
        """Returns the width of the given vehicle

        :param str vehicle_id: The ID of the vehicle we are interested in
        :return: Width of the vehicle in metres
        """
        raise NotImplementedError

    @abstractmethod
    def get_driving_distance(self, vehicle_id: str) -> float:
        """Returns the driving distance of the vehicle to the end of the current edge/road

        :param str vehicle_id: The ID of the vehicle we are interested in
        :return: The driving distance in metres
        """
        raise NotImplementedError

    @abstractmethod
    def get_speed(self, vehicle_id: str) -> float:
        """Returns the current speed of the given vehicle

        :param str vehicle_id: The ID of the vehicle we are interested in
        :return: The speed in metres/second
        """
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
        :param float to: The speed (in m/s) you would like the desired speed to be
            set to. If set to -1, the vehicle's speed will be controlled by
            the environment.
        """
        raise NotImplementedError

    @abstractmethod
    def get_position(self, vehicle_id) -> Tuple[float, float]:
        """Returns the current position of the given vehicle

        :param str vehicle_id: The ID of the vehicle we are interested in
        :return: The (x,y) position of the vehicle
        """
        raise NotImplementedError

    @abstractmethod
    def get_direction(self, vehicle_id) -> float:
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

        :param str vehicle_id: The ID of the vehicle we want to get the heading of
        :return: The direction of the vehicle in radians
        """
        raise NotImplementedError

    @abstractmethod
    def get_speed_limit(self, vehicle_id) -> float:
        """Returns the maximum speed the given vehicle should be travelling at

        This will usually be determined by the speed limit of the road the vehicle
        is currently on
        """
        raise NotImplementedError
