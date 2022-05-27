from __future__ import annotations

import math
from typing import Dict, Tuple, FrozenSet, Optional, Set

import numpy as np
from intersection_control.algorithms.stip.constants import VehicleState, MessageType
from intersection_control.communication import DistanceBasedUnit
from intersection_control.core import Vehicle, Environment, Message
from intersection_control.core.environment import Trajectory


class STIPVehicle(Vehicle):
    INTERSECTION_GRANULARITY = 10
    RECALCULATE_THRESHOLD = 0.5
    COMMUNICATION_RANGE = 75

    def __init__(self, vehicle_id: str, environment: Environment):
        super().__init__(vehicle_id, environment)
        self.messaging_unit = DistanceBasedUnit(self.get_id(), 75, self.get_position)
        self.state = VehicleState.EXIT
        self.approaching_intersection: Optional[Intersection] = None
        self.trajectory: Optional[Trajectory] = None
        self.arrived_at: Optional[float] = None
        self.target_speed = self.get_speed()
        self.cached_cells: Optional[Tuple[float, Set[Tuple[int, int]]]] = None

    def step(self):
        if self.state == VehicleState.APPROACH:
            if self.in_intersection():
                self.transition_to_enter()
        elif self.state == VehicleState.ENTER:
            if self.departing():
                self.transition_to_exit()
        elif self.state == VehicleState.EXIT:
            if self.approaching():
                self.transition_to_approach()

        self.sender_logic()
        for message in self.messaging_unit.receive():
            self.receiver_logic(message)

    def receiver_logic(self, safety_message: Message):
        c = safety_message.contents
        if self.state == VehicleState.APPROACH:
            if c["type"] == MessageType.ENTER or c["type"] == MessageType.CROSS:
                if self.does_overlap_with(c):
                    if not self.has_priority(c):
                        self.set_target_speed_to_miss_vehicle(c)

    def sender_logic(self):
        if self.state == VehicleState.APPROACH or self.state == VehicleState.WAIT:
            self.messaging_unit.broadcast(self.enter_message())
        elif self.state == VehicleState.ENTER:
            self.messaging_unit.broadcast(self.cross_message())
        elif self.state == VehicleState.EXIT:
            self.messaging_unit.broadcast(self.exit_message())

    def enter_message(self) -> Message:
        return Message(self.messaging_unit.address, {
            "type": MessageType.ENTER,
            "id": self.get_id(),
            "arrival_time": self.approximate_arrival_time(),
            "exit_time": self.approximate_exit_time(),
            "trajectory_cells_list": self.get_trajectory_cells_list()
        })

    def cross_message(self) -> Message:
        return Message(self.messaging_unit.address, {
            "type": MessageType.CROSS,
            "id": self.get_id(),
            "arrival_time": self.arrived_at,
            "exit_time": self.approximate_exit_time(),
            "trajectory_cells_list": self.get_trajectory_cells_list()
        })

    def exit_message(self) -> Message:
        return Message(self.messaging_unit.address, {
            "type": MessageType.EXIT,
            "id": self.get_id()
        })

    def approximate_arrival_time(self) -> float:
        if self.arrived_at is not None:
            return self.arrived_at
        driving_distance = self.get_driving_distance()
        target_speed = self.target_speed if self.target_speed is not None else self.get_speed()
        return self.environment.get_current_time() + driving_distance / target_speed

    def approximate_exit_time(self):
        return self.approximate_arrival_time() + self.trajectory.get_length() / self.trajectory.speed_limit

    def get_trajectory_cells_list(self):
        assert self.approaching_intersection is not None
        if self.cached_cells:
            arrival_time, cells = self.cached_cells
            if abs(self.approximate_arrival_time() - arrival_time) < self.RECALCULATE_THRESHOLD:
                return cells
        v = InternalVehicle(self.trajectory.speed_limit, self.get_length(), self.get_width(), self.get_trajectory(),
                            self.approaching_intersection)
        cells = set()
        while v.is_in_intersection():
            cells = cells.union(self.approaching_intersection.get_tiles_for_vehicle(v, (2, 2)))
            v.update(0.25)

        self.cached_cells = self.approximate_arrival_time(), cells
        return cells

    def transition_to_enter(self):
        self.state = VehicleState.ENTER
        self.set_desired_speed(self.trajectory.speed_limit)
        self.arrived_at = self.environment.get_current_time()

    def transition_to_exit(self):
        self.approaching_intersection = None
        self.trajectory = None
        self.cached_cells = None
        self.arrived_at = None
        self.state = VehicleState.EXIT
        self.set_desired_speed(-1)

    def transition_to_approach(self):
        intersection_id = self.approaching()
        self.approaching_intersection = Intersection(self.environment.intersections.get_width(intersection_id),
                                                     self.environment.intersections.get_height(intersection_id),
                                                     self.INTERSECTION_GRANULARITY,
                                                     self.environment.intersections.get_trajectories(intersection_id))
        self.trajectory = self.environment.intersections.get_trajectories(intersection_id)[self.get_trajectory()]
        self.state = VehicleState.APPROACH

    def does_overlap_with(self, c):
        space_overlaps = len(self.get_trajectory_cells_list().intersection(c["trajectory_cells_list"])) > 0
        time_overlaps = min(self.approximate_exit_time(), c["exit_time"]) - max(self.approximate_arrival_time(),
                                                                                c["arrival_time"]) > 0
        return space_overlaps and time_overlaps

    def has_priority(self, c):
        t = self.approximate_arrival_time()
        return t < c["arrival_time"] or (t == c["arrival_time"] and self.get_id() < c["id"])

    def set_target_speed_to_miss_vehicle(self, c):
        time_to_arrive = c["exit_time"] - self.environment.get_current_time()
        speed_to_miss = self.get_driving_distance() / time_to_arrive
        if speed_to_miss < self.target_speed:
            self.target_speed = speed_to_miss
            self.set_desired_speed(self.target_speed)

    def destroy(self):
        self.messaging_unit.destroy()


class InternalVehicle:
    def __init__(self, velocity, length, width, trajectory, intersection: Intersection):
        self.velocity = velocity
        self.acceleration = 0
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
