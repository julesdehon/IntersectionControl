from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Dict, Tuple
import numpy as np


@dataclass
class Trajectory:
    """Class to represent a trajectory through an intersection

    A trajectory is characterised by a speed limit, and a numpy array of
    consecutive x,y points along the trajectory. For example,
    [(-10,0), (10,0)] is a straight-line horizontal trajectory through the
    intersection"""
    speed_limit: float
    points: List[np.ndarray]


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
