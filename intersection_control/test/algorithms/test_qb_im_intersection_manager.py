import unittest
from typing import Dict, Tuple, List
from unittest.mock import MagicMock, patch
from callee import Captor
import math

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms
import numpy as np

from intersection_control.communication.distance_based_unit import DistanceBasedUnit
from intersection_control.core import Environment
from intersection_control.core.environment import VehicleHandler, IntersectionHandler
from intersection_control.core.intersection_manager import Trajectory
from intersection_control.core.communication import Message
from intersection_control.algorithms.qb_im.qb_im_intersection_manager import Intersection, InternalVehicle, \
    QBIMIntersectionManager
from intersection_control.algorithms.qb_im.constants import VehicleMessageType, IMMessageType


class TestIntersection(unittest.TestCase):
    def setUp(self) -> None:
        trajectories = {
            "SN": Trajectory(10, [np.array((10., -30.)), np.array((10., 30.))]),
            "NS": Trajectory(10, [np.array((-10., 30.)), np.array((-10., -30.))]),
            "EW": Trajectory(10, [np.array((30., 10.)), np.array((-30., 10.))]),
            "WE": Trajectory(10, [np.array((-30., -10.)), np.array((30., -10.))]),
            "WN": Trajectory(10, [np.array((-30., -10.)), np.array((-8., -4.)),
                                  np.array((6., 12.)), np.array((10., 30.))])
        }
        self.intersection = Intersection(60, 60, 10, trajectories)
        self.vehicle = InternalVehicle(10,  # velocity
                                       5,  # length
                                       2,  # width
                                       "WN",  # arrival_lane
                                       self.intersection)  # intersection

    def test_get_tiles_for_vehicle(self):
        draw(self.intersection, self.vehicle)


class TestIntersectionManager(unittest.TestCase):
    def setUp(self) -> None:
        self.env = FakeEnv()
        self.im = QBIMIntersectionManager("intersection", self.env, 40, 0.05)
        self.im.messaging_unit.send = MagicMock()
        self.messaging_unit = DistanceBasedUnit("test", 100, lambda: (0, 0))

    def test_handles_single_reservation(self):
        self.messaging_unit.send(self.im.messaging_unit.address, Message(self.messaging_unit.address, {
            "type": VehicleMessageType.REQUEST,
            "vehicle_id": "Bob",
            "arrival_time": 3,
            "arrival_lane": "WE",
            "turn": "s",
            "arrival_velocity": 6.5,
            "maximum_velocity": 11,
            "maximum_acceleration": 5,
            "minimum_acceleration": -2,
            "vehicle_length": 5,
            "vehicle_width": 2,
            "front_wheel_displacement": 1,
            "rear_wheel_displacement": 1,
            "max_steering_angle": math.radians(45),
            "max_turn_per_second": math.radians(10),
            "emergency": False
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
            "turn": "s",
            "arrival_velocity": 6.5,
            "maximum_velocity": 11,
            "maximum_acceleration": 5,
            "minimum_acceleration": -2,
            "vehicle_length": 5,
            "vehicle_width": 2,
            "front_wheel_displacement": 1,
            "rear_wheel_displacement": 1,
            "max_steering_angle": math.radians(45),
            "max_turn_per_second": math.radians(10),
            "emergency": False
        }))
        self.im.step()
        self.im.messaging_unit.send.assert_called_once()

        self.messaging_unit.send(self.im.messaging_unit.address, Message(rejected_vehicle_unit.address, {
            "type": VehicleMessageType.REQUEST,
            "vehicle_id": "Pat",
            "arrival_time": 3,
            "arrival_lane": "WE",
            "turn": "s",
            "arrival_velocity": 6.5,
            "maximum_velocity": 11,
            "maximum_acceleration": 5,
            "minimum_acceleration": -2,
            "vehicle_length": 5,
            "vehicle_width": 2,
            "front_wheel_displacement": 1,
            "rear_wheel_displacement": 1,
            "max_steering_angle": math.radians(45),
            "max_turn_per_second": math.radians(10),
            "emergency": False
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
            "SN": Trajectory(10, [np.array((10., -30.)), np.array((10., 30.))]),
            "NS": Trajectory(10, [np.array((-10., 30.)), np.array((-10., -30.))]),
            "EW": Trajectory(10, [np.array((30., 10.)), np.array((-30., 10.))]),
            "WE": Trajectory(10, [np.array((-30., -10.)), np.array((30., -10.))])
        }


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


def draw(intersection: Intersection, vehicle: InternalVehicle):
    def animate(_):
        for p in list(ax.patches):
            p.remove()
        x, y = vehicle.position
        x -= vehicle.width / 2
        y -= vehicle.length / 2
        car = patches.Rectangle((x, y), vehicle.width, vehicle.length, linewidth=1, edgecolor='r',
                                facecolor='none', zorder=5)
        vehicle_direction = vehicle.get_direction_vector()
        angle = np.arctan2(vehicle_direction[1], vehicle_direction[0])
        t = transforms.Affine2D().rotate_around(vehicle.position[0], vehicle.position[1],
                                                angle - math.radians(90)) + ax.transData
        car.set_transform(t)

        ax.add_patch(car)

        tiles = intersection.get_tiles_for_vehicle(vehicle, (2, 2))
        for tile in tiles:
            color_tile(ax, tile, intersection)

        vehicle.update(0.25)

    fig, ax = plt.subplots()
    ax.axis('equal')
    ax.spines['left'].set_position('zero')
    ax.spines['right'].set_color('none')
    ax.spines['bottom'].set_position('zero')
    ax.spines['top'].set_color('none')
    ax.xaxis.set_ticks_position('bottom')
    ax.yaxis.set_ticks_position('left')
    ax.set_xticks(np.arange(-intersection.width / 2, intersection.width / 2,
                            intersection.width / intersection.granularity))
    ax.set_yticks(np.arange(-intersection.height / 2, intersection.height / 2,
                            intersection.height / intersection.granularity))
    ax.set_xlim((-intersection.width / 2 - 1, intersection.width / 2 + 1))
    ax.set_ylim((-intersection.height / 2 - 1, intersection.height / 2 + 1))
    ax.grid()

    FuncAnimation(fig, animate, frames=20, interval=500, repeat=False)

    plt.show()


def color_tile(ax, tile, intersection):
    i, j = tile
    x_coord = (i * (intersection.width / intersection.granularity)) - intersection.width / 2
    y_coord = (j * (intersection.height / intersection.granularity)) - intersection.height / 2
    rect = patches.Rectangle((x_coord, y_coord), intersection.width / intersection.granularity,
                             intersection.height / intersection.granularity, linewidth=1, edgecolor='none',
                             facecolor='gray', zorder=1)
    ax.add_patch(rect)


if __name__ == '__main__':
    unittest.main()
