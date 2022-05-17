import unittest
import numpy as np
from intersection_control.environments.sumo.sumo_intersection_handler import PointBasedTrajectory


class TestTrajectory(unittest.TestCase):
    def test_speed_limit_correctly_set(self):
        speed_limit = 5
        trajectory = PointBasedTrajectory(speed_limit, [np.array([0., 0.]), np.array([0., 10.])])
        self.assertEqual(trajectory.speed_limit, speed_limit)

    def test_can_move_along_straight_trajectory(self):
        trajectory = PointBasedTrajectory(5, [np.array([0., 0.]), np.array([0., 10.])])
        end, _ = trajectory.point_at(5)
        self.assertTrue((end == np.array([0., 5.])).all())

    def test_angle_correct_for_straight_line_trajectory(self):
        trajectory = PointBasedTrajectory(5, [np.array([0., 0.]), np.array([0., 10.])])
        _, angle = trajectory.point_at(5)
        self.assertEqual(angle, np.pi / 2)

    def test_can_move_along_diagonal_trajectory(self):
        trajectory = PointBasedTrajectory(5, [np.array([0., 0.]), np.array([10., 10.])])
        x, y = 5., 5.
        end, angle = trajectory.point_at(np.sqrt(x ** 2 + y ** 2))
        self.assertTrue((end == np.array([x, y])).all())
        self.assertEqual(angle, np.pi / 4)

    def test_moves_along_trajectory_twice(self):
        trajectory = PointBasedTrajectory(5, [np.array([0., 0.]), np.array([10., 10.])])
        x1, y1 = 2., 2.
        end, angle1 = trajectory.point_at(np.sqrt(x1 ** 2 + y1 ** 2))
        self.assertTrue((end == np.array([x1, y1])).all())
        x2, y2 = 7., 7.
        end, angle2 = trajectory.point_at(np.sqrt(x2 ** 2 + y2 ** 2))
        self.assertTrue((end == np.array([x2, y2])).all())
        self.assertEqual(angle1, angle2)

    def test_moves_along_curved_trajectory(self):
        trajectory = PointBasedTrajectory(5, [np.array([0., 0.]), np.array([10., 0.]), np.array([10., 10.])])
        end, angle = trajectory.point_at(5)
        self.assertTrue((end == np.array([5., 0.])).all())
        self.assertEqual(angle, 0)
        end, angle = trajectory.point_at(15)
        self.assertTrue((end == np.array([10., 5.])).all())
        self.assertEqual(angle, np.pi / 2)

    def test_moves_until_end_of_trajectory(self):
        trajectory = PointBasedTrajectory(5, [np.array([0., 0.]), np.array([0., 10.])])
        end, _ = trajectory.point_at(1000)
        self.assertTrue((end == np.array([0., 10.])).all())
