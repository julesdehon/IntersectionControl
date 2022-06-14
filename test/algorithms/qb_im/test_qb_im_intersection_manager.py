import unittest
from typing import Dict, Tuple, List, Set
from unittest.mock import MagicMock, patch
from callee import Captor
import math

import numpy as np

from intersection_control.communication.distance_based_unit import DistanceBasedUnit
from intersection_control.core import Environment
from intersection_control.core.environment import VehicleHandler, IntersectionHandler
from intersection_control.core.algorithm.intersection_manager import Trajectory
from intersection_control.core.communication import Message
from intersection_control.algorithms.qb_im.qb_im_intersection_manager import QBIMIntersectionManager
from intersection_control.algorithms.utils.discretised_intersection import InternalVehicle, Intersection
from intersection_control.algorithms.qb_im.constants import VehicleMessageType, IMMessageType
from intersection_control.environments.sumo.sumo_intersection_handler import PointBasedTrajectory


class TestIntersection(unittest.TestCase):
    def setUp(self) -> None:
        trajectories = {
            "SN": PointBasedTrajectory(10, [np.array((10., -30.)), np.array((10., 30.))]),
            "NS": PointBasedTrajectory(10, [np.array((-10., 30.)), np.array((-10., -30.))]),
            "EW": PointBasedTrajectory(10, [np.array((30., 10.)), np.array((-30., 10.))]),
            "WE": PointBasedTrajectory(10, [np.array((-30., -10.)), np.array((30., -10.))]),
            "WN": PointBasedTrajectory(10, [np.array((-30., -10.)), np.array((-8., -4.)),
                                            np.array((6., 12.)), np.array((10., 30.))])
        }
        self.intersection = Intersection(60, 60, (0, 0), 20, trajectories)

    def test_tiles_for_straight_trajectory_are_along_the_same_axis(self):
        vehicle = InternalVehicle(10,  # velocity
                                  5,  # length
                                  2,  # width
                                  "NS",  # arrival_lane
                                  self.intersection)  # intersection
        all_traversed_tiles = []
        while vehicle.is_in_intersection():
            all_traversed_tiles.append(self.intersection.get_tiles_for_vehicle(vehicle, (2, 2)))
            vehicle.update(0.25)
        x_coords = [{x for (x, _) in tiles} for tiles in all_traversed_tiles]
        for i in range(len(x_coords) - 1):
            self.assertSetEqual(x_coords[i], x_coords[i + 1])

    def test_tiles_at_each_timestep_are_adjacent(self):
        vehicle = InternalVehicle(10,  # velocity
                                  5,  # length
                                  2,  # width
                                  "WN",  # arrival_lane
                                  self.intersection)  # intersection

        def adjacent(tile1, tile2):
            (x1, y1) = tile1
            (x2, y2) = tile2
            return (abs(x1 - x2) == 1) ^ (abs(y2 - y1) == 1)

        while vehicle.is_in_intersection():
            tiles = self.intersection.get_tiles_for_vehicle(vehicle, (2, 2))
            for tile in tiles:
                # Tile is adjacent to at least one other
                self.assertTrue(any([adjacent(tile, other) for other in tiles]))
            vehicle.update(0.25)


class TestIntersectionManager(unittest.TestCase):
    def setUp(self) -> None:
        self.env = FakeEnv()
        self.im = QBIMIntersectionManager("intersection", self.env, 40, 0.05,
                                          DistanceBasedUnit("intersection", 75,
                                                            lambda: self.env.intersections.get_position(
                                                                "intersection")))
        self.im.messaging_unit.send = MagicMock()
        self.messaging_unit = DistanceBasedUnit("test", 100, lambda: (0, 0))

    def test_handles_single_reservation(self):
        self.messaging_unit.send(self.im.messaging_unit.address, Message(self.messaging_unit.address, {
            "type": VehicleMessageType.REQUEST,
            "vehicle_id": "Bob",
            "arrival_time": 3,
            "arrival_lane": "WE",
            "arrival_velocity": 6.5,
            "maximum_acceleration": 5,
            "maximum_velocity": 11,
            "vehicle_length": 5,
            "vehicle_width": 2,
            "distance": 10,
        }))
        self.im.step()
        captured_message = Captor()
        self.im.messaging_unit.send.assert_called_with(self.messaging_unit.address, captured_message)
        self.assertEqual(captured_message.arg.sender, self.im.messaging_unit.address)
        self.assertEqual(captured_message.arg.contents["type"], IMMessageType.CONFIRM)

    def test_rejects_conflicting_reservation(self):
        rejected_vehicle_unit = DistanceBasedUnit("reject", 100, lambda: (0, 0))

        self.messaging_unit.send(self.im.messaging_unit.address, Message(self.messaging_unit.address, {
            "type": VehicleMessageType.REQUEST,
            "vehicle_id": "Bob",
            "arrival_time": 3,
            "arrival_lane": "WE",
            "arrival_velocity": 6.5,
            "maximum_acceleration": 5,
            "maximum_velocity": 11,
            "vehicle_length": 5,
            "vehicle_width": 2,
            "distance": 10,
        }))
        self.im.step()
        self.im.messaging_unit.send.assert_called_once()

        self.messaging_unit.send(self.im.messaging_unit.address, Message(rejected_vehicle_unit.address, {
            "type": VehicleMessageType.REQUEST,
            "vehicle_id": "Pat",
            "arrival_time": 3,
            "arrival_lane": "WE",
            "arrival_velocity": 6.5,
            "maximum_acceleration": 5,
            "maximum_velocity": 11,
            "vehicle_length": 5,
            "vehicle_width": 2,
            "distance": 10,
        }))
        self.im.step()
        captured_message = Captor()
        self.im.messaging_unit.send.assert_called_with(rejected_vehicle_unit.address, captured_message)
        self.assertEqual(captured_message.arg.sender, self.im.messaging_unit.address)
        self.assertEqual(captured_message.arg.contents["type"], IMMessageType.REJECT)


#################################
# Utility classes and functions #
#################################


class FakeIntersectionHandler(IntersectionHandler):
    def get_ids(self) -> List[str]:
        return ["intersection"]

    def get_width(self, intersection_id: str) -> float:
        return 60

    def get_height(self, intersection_id: str) -> float:
        return 60

    def get_position(self, intersection_id: str) -> Tuple[float, float]:
        return 0, 0

    def get_trajectories(self, intersection_id: str) -> Dict[str, Trajectory]:
        return {
            "SN": PointBasedTrajectory(10, [np.array((10., -30.)), np.array((10., 30.))]),
            "NS": PointBasedTrajectory(10, [np.array((-10., 30.)), np.array((-10., -30.))]),
            "EW": PointBasedTrajectory(10, [np.array((30., 10.)), np.array((-30., 10.))]),
            "WE": PointBasedTrajectory(10, [np.array((-30., -10.)), np.array((30., -10.))])
        }

    def set_traffic_light_phase(self, intersection_id: str, phase: Tuple[Set[str], Set[str], Set[str]]):
        pass


class FakeEnv(Environment):
    @patch.multiple(VehicleHandler, __abstractmethods__=set())
    def __init__(self):
        self.t = 0
        self._intersections = FakeIntersectionHandler()
        self._vehicles = VehicleHandler()

    @property
    def intersections(self) -> IntersectionHandler:
        return self._intersections

    @property
    def vehicles(self) -> VehicleHandler:
        return self._vehicles

    def get_current_time(self) -> float:
        t = self.t
        self.t += 1
        return t

    def step(self):
        pass

    def get_removed_vehicles(self) -> List[str]:
        return []

    def get_added_vehicles(self) -> List[str]:
        return []

    def clear(self):
        pass


if __name__ == '__main__':
    unittest.main()
