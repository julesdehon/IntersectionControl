from __future__ import annotations
from typing import Dict, Tuple, FrozenSet, Set
import logging

from intersection_control.algorithms.utils.discretised_intersection import InternalVehicle, Intersection
from intersection_control.core import IntersectionManager, MessagingUnit
from intersection_control.core import Message, Environment
from intersection_control.algorithms.qb_im.constants import IMMessageType, VehicleMessageType
import numpy as np
import math

logger = logging.getLogger(__name__)

TIME_BUFFER = 0.5
EDGE_TILE_TIME_BUFFER = 1
SAFETY_BUFFER = (0.5, 1)
MUST_ACCELERATE_THRESHOLD = 4  # Vehicles travelling slower than this threshold must accelerate through the intersection


class QBIMIntersectionManager(IntersectionManager):
    def __init__(self, intersection_id: str, environment: Environment, granularity: int, time_discretisation: float,
                 messaging_unit: MessagingUnit):
        super().__init__(intersection_id, environment)
        self.messaging_unit = messaging_unit
        self.time_discretisation = time_discretisation
        self.tiles: Dict[Tuple[Tuple[int, int], float], str] = {}  # A map from tiles and times to vehicle ids
        # A map from vehicles to sets of tiles
        self.reservations: Dict[str, Set[Tuple[float, FrozenSet[Tuple[int, int]]]]] = {}
        self.timeouts = {}  # A map from vehicles to times
        self.intersection = Intersection(self.get_width(),
                                         self.get_height(),
                                         self.get_position(),
                                         granularity,
                                         self.get_trajectories())
        self.d = {trajectory[0]: np.inf for trajectory in self.get_trajectories()}

    def step(self):
        for message in self.messaging_unit.receive():
            self.handle_message(message)

    def handle_message(self, message: Message):
        if message.contents["type"] == VehicleMessageType.REQUEST \
                or message.contents["type"] == VehicleMessageType.CHANGE_REQUEST:
            self.handle_request_message(message)
        elif message.contents["type"] == VehicleMessageType.DONE:
            logger.debug(f"[{self.environment.get_current_time()}] Received done message from {message.sender}")
            self.handle_done_message(message)
        else:
            logger.warning(f"[{self.environment.get_current_time()}] Received unknown message type "
                           f"from {message.sender}. Ignoring.")

    def handle_request_message(self, message: Message):
        assert message.contents["type"] == VehicleMessageType.REQUEST \
               or message.contents["type"] == VehicleMessageType.CHANGE_REQUEST

        # If it is a change request, delete all knowledge about the request
        if message.contents["type"] == VehicleMessageType.CHANGE_REQUEST:
            old_tile_times = self.reservations[message.sender]
            for (time, tiles) in old_tile_times:
                for tile in tiles:
                    self.tiles.pop((tile, time))
            self.reservations.pop(message.sender)

        # If the vehicle is still on timeout, reject the request
        curr_time = self.discretise_time(self.environment.get_current_time())
        if message.sender in self.timeouts and self.timeouts[message.sender] > curr_time:
            logger.debug(f"[{self.environment.get_current_time()}] Rejecting request for {message.sender}: "
                         f"timeout not yet served")
            self.messaging_unit.send(message.sender, Message(self.messaging_unit.address, {
                "type": IMMessageType.REJECT,
                "timeout": self.timeouts[message.sender]
            }))
            return

        arrival_time = message.contents["arrival_time"]
        self.timeouts[message.sender] = curr_time + min(0.5, (arrival_time - curr_time) / 2)

        # If farther than nearest rejected vehicle, reject the request
        if message.contents["distance"] > self.d[message.contents["arrival_lane"][0]]:
            logger.debug(f"[{self.environment.get_current_time()}] Rejecting request for {message.sender}: "
                         f"farther away than nearest waiting vehicle")
            self.messaging_unit.send(message.sender, Message(self.messaging_unit.address, {
                "type": IMMessageType.REJECT,
                "timeout": self.timeouts[message.sender]
            }))
            return

        for acceleration in [True, False]:
            tile_times = set()
            time = self.discretise_time(arrival_time)
            temp_vehicle = InternalVehicle(message.contents["arrival_velocity"],
                                           message.contents["vehicle_length"],
                                           message.contents["vehicle_width"],
                                           message.contents["arrival_lane"],
                                           self.intersection,
                                           acceleration=message.contents["maximum_acceleration"] if acceleration else 0)
            no_collisions = True
            while temp_vehicle.is_in_intersection():
                # TODO: Tune safety buffer
                occupied_tiles = self.intersection.get_tiles_for_vehicle(temp_vehicle, SAFETY_BUFFER)
                tile_times.add((time, occupied_tiles))
                for tile in occupied_tiles:
                    buf = TIME_BUFFER  # TODO: Tune this - the time buffer around which reservation slots are checked
                    if tile[0] == 0 or tile[1] == 0 or tile[0] == self.intersection.granularity - 1 or \
                            tile[1] == self.intersection.granularity - 1:
                        buf = EDGE_TILE_TIME_BUFFER
                    for i in np.arange(-buf, buf, self.time_discretisation):
                        if (tile, time + i) in self.tiles:
                            if acceleration and message.contents["arrival_velocity"] > MUST_ACCELERATE_THRESHOLD:
                                no_collisions = False
                                break
                            else:
                                logger.debug(f"[{self.environment.get_current_time()}] Rejecting request for "
                                             f"{message.sender}: reservation collision")
                                self.messaging_unit.send(message.sender, Message(self.messaging_unit.address, {
                                    "type": IMMessageType.REJECT,
                                    "timeout": self.timeouts[message.sender]
                                }))
                                self.d[message.contents["arrival_lane"][0]] = message.contents["distance"]
                                return
                    if not no_collisions:
                        break
                if not no_collisions:
                    break
                time = self.discretise_time(time + self.time_discretisation)  # Make 100% sure it is discretised
                temp_vehicle.update(self.time_discretisation)
                if temp_vehicle.velocity >= min(
                        message.contents["maximum_velocity"], temp_vehicle.trajectory.speed_limit):
                    temp_vehicle.acceleration = 0

            if not no_collisions:
                continue

            for (time, tiles) in tile_times:
                for tile in tiles:
                    self.tiles[(tile, time)] = message.sender
            self.reservations[message.sender] = tile_times
            logger.debug(f"[{self.environment.get_current_time()}] Accepting request for {message.sender}")
            self.messaging_unit.send(message.sender, Message(self.messaging_unit.address, {
                "type": IMMessageType.CONFIRM,
                "reservation_id": message.sender,
                "arrival_time": arrival_time,
                "arrival_velocity": message.contents["arrival_velocity"],
                "early_error": arrival_time - TIME_BUFFER,
                "late_error": arrival_time + TIME_BUFFER,
                "accelerate": acceleration
            }))
            self.d[message.contents["arrival_lane"][0]] = np.inf
            break

    def handle_done_message(self, message: Message):
        assert message.contents["type"] == VehicleMessageType.DONE
        old_tile_times = self.reservations[message.sender]
        for (time, tiles) in old_tile_times:
            for tile in tiles:
                self.tiles.pop((tile, time))
        self.reservations.pop(message.sender)

    def discretise_time(self, time, direction="nearest"):
        if direction == "ceiling":
            f = math.ceil
        elif direction == "floor":
            f = math.floor
        else:
            f = round
        num_decimals = str(self.time_discretisation)[::-1].find(".")
        return round(self.time_discretisation * f(time / self.time_discretisation), num_decimals)
