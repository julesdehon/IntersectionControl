from abc import ABC, abstractmethod
from typing import List, Dict, Tuple, Set


class Trajectory(ABC):
    """Class to represent a trajectory through an intersection

    A trajectory has a speed limit, and can be moved along. Different environments
    may implement trajectories in different ways, and this API should cater to
    most all of them
    """
    @property
    @abstractmethod
    def speed_limit(self) -> float:
        raise NotImplementedError

    @abstractmethod
    def point_at(self, distance: float) -> Tuple[Tuple[float, float], float]:
        raise NotImplementedError

    @abstractmethod
    def get_length(self) -> float:
        raise NotImplementedError

    def get_starting_position(self) -> Tuple[Tuple[float, float], float]:
        return self.point_at(0)


class IntersectionHandler(ABC):
    @abstractmethod
    def get_ids(self) -> List[str]:
        """Return the ids of all intersections in the environment"""
        raise NotImplementedError

    @abstractmethod
    def get_width(self, intersection_id: str) -> float:
        """Return the width of the given intersection in metres

        :param str intersection_id: The id of the intersection you want the
            width of
        :return: The width of the given intersection in metres
        """
        raise NotImplementedError

    @abstractmethod
    def get_height(self, intersection_id: str) -> float:
        """Return the height of the given intersection in metres

        :param str intersection_id: The id of the intersection you want the
            height of
        :return: The height of the given intersection in metres
        """
        raise NotImplementedError

    @abstractmethod
    def get_position(self, intersection_id: str) -> Tuple[float, float]:
        """Return the position of the centre of the given intersection

        :param str intersection_id: The id of the intersection you want the
            position of
        :return: (x,y) the position of the centre of the given intersection in
            the environment's coordinate space
        """
        raise NotImplementedError

    @abstractmethod
    def get_trajectories(self, intersection_id: str) -> Dict[str, Trajectory]:
        """Return all the possible trajectories through the given intersection

        :param str intersection_id: The id of the intersection you want the
            trajectories for
        :return: A dictionary mapping trajectory ids to trajectories
        """
        raise NotImplementedError

    @abstractmethod
    def set_traffic_light_phase(self, intersection_id: str, phase: Tuple[Set[str], Set[str], Set[str]]):
        """Set the traffic lights at the given intersection to the phase described by phase

        :param str intersection_id:
        :param Tuple[Set[str], Set[str], Set[str]] phase: A tuple of length 3: (Green, Yellow, Red)  where each of
            Green, Yellow and Red are sets of trajectory IDs such that all the trajectories in Red will be shown a red
            light, and the same for Yellow and Green
        """
        raise NotImplementedError
