from __future__ import annotations
import math
from typing import Tuple, Dict, FrozenSet
from shapely.geometry import Polygon

import numpy as np

from intersection_control.core.environment import Trajectory


class InternalVehicle:
    def __init__(self, velocity, length, width, trajectory, intersection: Intersection, acceleration=0):
        self.velocity = velocity
        self.acceleration = acceleration
        self.length = length
        self.width = width
        self.intersection = intersection
        self.trajectory = intersection.trajectories[trajectory]
        self.position, self.angle = self.trajectory.get_starting_position()
        self.distance_moved = 0

    def is_in_intersection(self):
        return self.distance_moved < self.trajectory.get_length()

    def update(self, dt):
        self.distance_moved = self.distance_moved + self.velocity * dt
        self.position, self.angle = self.trajectory.point_at(self.distance_moved)
        self.velocity += self.acceleration * dt


class Intersection:
    """
    A class to represent an intersection, as perceived by the Query-Based Intersection Manager

    Attributes
    ----------
    grid : np.ndarray (granularity, granularity)
        The discretised grid that forms the intersection
    granularity : int
        Determines how precisely the area of the intersection will be discretised
    width: float
        The width of the intersection
    height: float
        The height of the intersection
    trajectories: Dict[str, [np.ndarray]]
        A mapping from trajectory name to trajectory. A trajectory is characterised by a list of positions along that
        trajectory. Vehicle movement along those trajectories can then be interpolated between those points.
    """

    def __init__(self, width: float, height: float, position: Tuple[float, float], granularity: int,
                 trajectories: Dict[str, Trajectory]):
        self.tile_shapes = [[Polygon([(i, j), (i, j + 1), (i + 1, j + 1), (i + 1, j)]) for j in range(granularity)]
                            for i in range(granularity)]
        self.granularity = granularity
        self.size = np.array([width, height])
        self.trajectories = trajectories
        self.position = position

    def get_tiles_for_vehicle(self, vehicle: InternalVehicle,
                              safety_buffer: Tuple[float, float]) -> FrozenSet[Tuple[int, int]]:
        # normalised perpendicular vectors
        v1 = np.array([np.cos(vehicle.angle), np.sin(vehicle.angle)])
        v2 = np.array([-v1[1], v1[0]])

        v1 *= (vehicle.length + safety_buffer[1]) / 2
        v2 *= (vehicle.width + safety_buffer[0]) / 2

        corners = np.array([
            vehicle.position + v1 + v2,
            vehicle.position - v1 + v2,
            vehicle.position - v1 - v2,
            vehicle.position + v1 - v2
        ])

        corners_transformed = np.array([
            (((corner - self.position) + self.size / 2) / self.size) * self.granularity for corner in corners
        ])
        vehicle_and_buffer = Polygon(corners_transformed)

        min_x = math.floor(np.min(corners_transformed[:, 0]))
        max_x = math.floor(np.max(corners_transformed[:, 0]))
        min_y = math.floor(np.min(corners_transformed[:, 1]))
        max_y = math.floor(np.max(corners_transformed[:, 1]))

        tile_coords = set()
        for i in range(max(min_x, 0), min(max_x + 1, self.granularity)):
            for j in range(max(min_y, 0), min(max_y + 1, self.granularity)):
                if vehicle_and_buffer.intersects(self.tile_shapes[i][j]):
                    tile_coords.add((i, j))

        return frozenset(tile_coords)

    @staticmethod
    def _is_point_in_rectangle(corners, point):
        """
        Solution adapted from StackOverflow answer:
        https://stackoverflow.com/questions/2752725/finding-whether-a-point-lies-inside-a-rectangle-or-not#:~:text=In%20any%20case%2C%20for%20any,test%20%2D%20the%20point%20is%20inside.
        """
        ab = corners[1] - corners[0]
        am = point - corners[0]

        bc = corners[2] - corners[1]
        bm = point - corners[1]
        return 0 <= np.dot(ab, am) <= np.dot(ab, ab) and 0 <= np.dot(bc, bm) <= np.dot(bc, bc)
