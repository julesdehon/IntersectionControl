from __future__ import annotations
from typing import Dict, Tuple, FrozenSet
import logging

from intersection_control.communication.distance_based_unit import DistanceBasedUnit
from intersection_control.core import IntersectionManager
from intersection_control.core import Message, Environment
from intersection_control.core.environment import Trajectory
from intersection_control.algorithms.qb_im.constants import IMMessageType, VehicleMessageType
import numpy as np
import math

logger = logging.getLogger(__name__)

TIME_BUFFER = 1


class QBIMIntersectionManager(IntersectionManager):
    def __init__(self, intersection_id: str, environment: Environment, granularity: int, time_discretisation: float):
        super().__init__(intersection_id, environment)
        self.messaging_unit = DistanceBasedUnit(self.intersection_id, 75, self.get_position)
        self.time_discretisation = time_discretisation
        self.tiles: Dict[((int, int), float), str] = {}  # A map from tiles and times to vehicle ids
        self.reservations = {}  # A map from vehicles to sets of tiles
        self.timeouts = {}  # A map from vehicles to times
        self.intersection = Intersection(self.get_width(),
                                         self.get_height(),
                                         granularity,
                                         self.get_trajectories())

    def step(self):
        for message in self.messaging_unit.receive():
            self.handle_message(message)

    def handle_message(self, message: Message):
        if message.contents["type"] == VehicleMessageType.REQUEST \
                or message.contents["type"] == VehicleMessageType.CHANGE_REQUEST:
            self.handle_request_message(message)
        elif message.contents["type"] == VehicleMessageType.DONE:
            logger.debug(f"Received done message from {message.sender}")
        else:
            logger.warning(f"Received unknown message type from {message.sender}. Ignoring.")

    def handle_request_message(self, message: Message):
        if message.contents["type"] == VehicleMessageType.CHANGE_REQUEST:
            old_tile_times = self.reservations[message.sender]
            for (time, tiles) in old_tile_times:
                for tile in tiles:
                    self.tiles.pop((tile, time))
            self.reservations.pop(message.sender)

        curr_time = self.discretise_time(self.environment.get_current_time())
        if message.sender in self.timeouts and self.timeouts[message.sender] > curr_time:
            logger.debug(f"Rejecting request for {message.sender}: timeout not yet served")
            self.messaging_unit.send(message.sender, Message(self.messaging_unit.address, {
                "type": IMMessageType.REJECT,
                "timeout": self.timeouts[message.sender]
            }))
            return
        arrival_time = message.contents["arrival_time"]
        self.timeouts[message.sender] = curr_time + min(0.5, (arrival_time - curr_time) / 2)

        tile_times = set()
        time = self.discretise_time(arrival_time)
        temp_vehicle = InternalVehicle(message.contents["arrival_velocity"],
                                       message.contents["vehicle_length"],
                                       message.contents["vehicle_width"],
                                       message.contents["arrival_lane"],
                                       self.intersection)

        while temp_vehicle.is_in_intersection():
            # TODO: Tune safety buffer
            occupied_tiles = self.intersection.get_tiles_for_vehicle(temp_vehicle, (2, 2))
            tile_times.add((time, occupied_tiles))
            for tile in occupied_tiles:
                buf = TIME_BUFFER  # TODO: Tune this - the time buffer around which reservation slots are checked
                for i in np.arange(-buf, buf, self.time_discretisation):
                    if (tile, time + i) in self.tiles:
                        logger.debug(f"Rejecting request for {message.sender}: reservation collision")
                        self.messaging_unit.send(message.sender, Message(self.messaging_unit.address, {
                            "type": IMMessageType.REJECT,
                            "timeout": self.timeouts[message.sender]
                        }))
                        return
            time += self.time_discretisation
            temp_vehicle.update(self.time_discretisation)

        for (time, tiles) in tile_times:
            for tile in tiles:
                self.tiles[(tile, time)] = message.sender
        self.reservations[message.sender] = tile_times
        logger.debug(f"Accepting request for {message.sender}")
        self.messaging_unit.send(message.sender, Message(self.messaging_unit.address, {
            "type": IMMessageType.CONFIRM,
            "reservation_id": message.sender,
            "arrival_time": arrival_time,
            "arrival_velocity": message.contents["arrival_velocity"],
            "early_error": arrival_time - TIME_BUFFER / 2,
            "late_error": arrival_time + TIME_BUFFER / 2
        }))

    def discretise_time(self, time, direction="nearest"):
        if direction == "ceiling":
            f = math.ceil
        elif direction == "floor":
            f = math.floor
        else:
            f = round
        return self.time_discretisation * f(time / self.time_discretisation)


class InternalVehicle:
    def __init__(self, velocity, length, width, trajectory, intersection: Intersection):
        self.velocity = velocity
        self.acceleration = 0
        self.length = length
        self.width = width
        self.intersection = intersection
        self.trajectory = np.copy(intersection.trajectories[trajectory].points)
        self.curr_trajectory_slice = 0
        self.position = np.copy(self.trajectory[self.curr_trajectory_slice])

    def is_in_intersection(self):
        intersection_shape = np.array([self.intersection.width, self.intersection.height])
        return ((-intersection_shape / 2 <= self.position) & (
                self.position <= intersection_shape / 2)).all() and self.curr_trajectory_slice < len(
            self.trajectory) - 1

    def update(self, dt):
        distance_moved = self.velocity * dt
        self.move_along_trajectory(distance_moved)
        self.velocity += self.acceleration * dt

    def move_along_trajectory(self, distance: float):
        while distance > 0 and self.curr_trajectory_slice < len(self.trajectory) - 1:
            vector_to_waypoint = self.get_direction_vector()
            dist_to_waypoint = np.sqrt(np.sum(vector_to_waypoint ** 2))
            dist_moved = min(distance, dist_to_waypoint)
            self.position += vector_to_waypoint * (dist_moved / dist_to_waypoint)
            distance -= dist_moved
            if (self.get_direction_vector() == [0, 0]).all():
                self.curr_trajectory_slice += 1

    def get_direction_vector(self):
        return self.trajectory[self.curr_trajectory_slice + 1] - self.position


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

    def __init__(self, width: float, height: float, granularity: int,
                 trajectories: Dict[str, Trajectory]):
        self.grid = np.full((granularity, granularity), False)
        self.granularity = granularity
        self.width = width
        self.height = height
        self.trajectories = trajectories

    def get_tiles_for_vehicle(self, vehicle: InternalVehicle,
                              safety_buffer: Tuple[float, float]) -> FrozenSet[Tuple[int, int]]:
        # normalised perpendicular vectors
        v1 = vehicle.get_direction_vector() / np.linalg.norm(vehicle.get_direction_vector())
        v2 = np.array([-v1[1], v1[0]])

        v1 *= (vehicle.length + safety_buffer[1]) / 2
        v2 *= (vehicle.width + safety_buffer[0]) / 2

        corners = np.array([
            vehicle.position + v1 + v2,
            vehicle.position - v1 + v2,
            vehicle.position - v1 - v2,
            vehicle.position + v1 - v2
        ])

        min_x = np.min(corners[:, 0])
        max_x = np.max(corners[:, 0])
        min_y = np.min(corners[:, 1])
        max_y = np.max(corners[:, 1])

        min_x_box = math.floor(((min_x + self.width / 2) / self.width) * self.granularity)
        max_x_box = math.floor(((max_x + self.width / 2) / self.width) * self.granularity)
        min_y_box = math.floor(((min_y + self.width / 2) / self.width) * self.granularity)
        max_y_box = math.floor(((max_y + self.width / 2) / self.width) * self.granularity)

        tile_coords = set()
        for i in range(min_x_box, max_x_box + 1):
            for j in range(min_y_box, max_y_box + 1):
                tile_coords.add((i, j))

        return frozenset(tile_coords)
