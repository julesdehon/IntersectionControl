from __future__ import annotations

from typing import Tuple, Optional, Set
import numpy as np

from intersection_control.algorithms.stip.constants import VehicleState, MessageType
from intersection_control.algorithms.utils.discretised_intersection import Intersection, InternalVehicle
from intersection_control.core import Vehicle, Environment, Message, MessagingUnit
from intersection_control.core.environment import Trajectory


class STIPVehicle(Vehicle):
    INTERSECTION_GRANULARITY = 30
    RECALCULATE_THRESHOLD = 0.5
    SAFETY_BUFFER = (0.5, 1)

    def __init__(self, vehicle_id: str, environment: Environment, messaging_unit: MessagingUnit):
        super().__init__(vehicle_id, environment)
        self.messaging_unit = messaging_unit
        self.state = VehicleState.EXIT
        self.approaching_intersection: Optional[Intersection] = None
        self.trajectory: Optional[Trajectory] = None
        self.arrived_at: Optional[float] = None
        self.target_speed: float = self.get_speed()
        self.cached_cells: Optional[Tuple[float, Set[Tuple[int, int]]]] = None
        self.last_sent_distance: Optional[float] = None

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

        self.last_sent_distance = self.get_driving_distance()

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
            "trajectory_cells_list": self.get_trajectory_cells_list(),
            "lane": self.get_trajectory()[0],
            "distance": self.get_driving_distance()
        })

    def cross_message(self) -> Message:
        return Message(self.messaging_unit.address, {
            "type": MessageType.CROSS,
            "id": self.get_id(),
            "arrival_time": self.arrived_at,
            "exit_time": self.approximate_exit_time(),
            "trajectory_cells_list": self.get_trajectory_cells_list(),
            "lane": self.get_trajectory()[0],
            "distance": 0
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
        target_speed = min(self.target_speed, self.get_speed())
        if target_speed == 0:
            target_speed = self.target_speed
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
            cells = cells.union(self.approaching_intersection.get_tiles_for_vehicle(v, self.SAFETY_BUFFER))
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
        self.last_sent_distance = None

    def transition_to_approach(self):
        intersection_id = self.approaching()
        self.approaching_intersection = Intersection(self.environment.intersections.get_width(intersection_id),
                                                     self.environment.intersections.get_height(intersection_id),
                                                     self.environment.intersections.get_position(intersection_id),
                                                     self.INTERSECTION_GRANULARITY,
                                                     self.environment.intersections.get_trajectories(intersection_id))
        self.trajectory = self.environment.intersections.get_trajectories(intersection_id)[self.get_trajectory()]
        self.state = VehicleState.APPROACH
        self.target_speed = self.get_speed()
        self.cached_cells = None
        self.arrived_at = None
        self.last_sent_distance = self.approximate_arrival_time()

    def does_overlap_with(self, c):
        space_overlaps = len(self.get_trajectory_cells_list().intersection(c["trajectory_cells_list"])) > 0
        time_overlaps = min(self.approximate_exit_time(), c["exit_time"]) - max(self.approximate_arrival_time(),
                                                                                c["arrival_time"]) > 0
        return space_overlaps and time_overlaps

    def has_priority(self, c):
        d = self.last_sent_distance
        return d < c["distance"] or (d == c["distance"] and self.get_id() < c["id"])

    def set_target_speed_to_miss_vehicle(self, c):
        time_to_arrive = c["exit_time"] - self.environment.get_current_time()
        speed_to_miss = self.get_driving_distance() / time_to_arrive
        if speed_to_miss < self.target_speed:
            self.target_speed = speed_to_miss
            self.set_desired_speed(self.target_speed)

    def destroy(self):
        self.messaging_unit.destroy()
