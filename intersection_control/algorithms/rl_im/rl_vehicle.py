from typing import Optional, List
import numpy as np

from intersection_control.communication.distance_based_unit import DistanceBasedUnit
from intersection_control.core import Vehicle, Environment, Message


class RLVehicle(Vehicle):
    def __init__(self, vehicle_id: str, environment: Environment, intersection_id: str):
        super().__init__(vehicle_id, environment)
        self.messaging_unit = DistanceBasedUnit(vehicle_id, 75, self.get_position)
        self.trajectory_list = list(self.environment.intersections.get_trajectories(intersection_id).keys())
        self.last_obs: Optional[List[List[float]]] = None

    def destroy(self):
        self.messaging_unit.destroy()

    def step(self):
        """Perform a single step in the simulation

        In training mode, this will simply broadcast messages to the surrounding vehicles
        with the necessary information - but in evaluation/deployment mode, this should
        use a trained agent to take actions according to the current observation.
        """
        self.messaging_unit.broadcast(Message(self.get_id(), {
            "position": np.array(self.get_position()),
            "trajectory": self.get_trajectory(),
            "direction": self.get_direction(),
            "speed": self.get_speed(),
            "timestamp": self.environment.get_current_time()
        }))

    def apply_action(self, action):
        """Applies the given action to the vehicle

        The action will be one of:
            0 = NOOP
            1 = ACCELERATE
            2 = DECELERATE

        :param action: The action to be performed
        """
        if action == 0:
            # noop
            return
        elif action == 1:
            # accelerate
            self.set_desired_speed(self.get_speed() * 1.1)
        elif action == 2:
            # decelerate
            self.set_desired_speed(self.get_speed() * 0.9)

    def get_observation(self):
        """Returns this vehicle's observation

        The observation space contains information about the ego vehicle, and its 5
        nearest neighbours:
            - x_distance
            - y_distance
            - speed
            - sin(direction)
            - cos(direction)
            - trajectory
            - present (1 if this is an actual vehicle, 0 if it is padding)

        :return: The observation for this agent at this time step
        """
        messages = [message for message in self.messaging_unit.receive() if
                    message.contents["timestamp"] == self.environment.get_current_time()]
        x, y = self.get_position()
        messages = sorted(messages,
                          key=lambda m: np.linalg.norm(m.contents["position"] - np.array([x, y])))
        messages = messages[:5]
        direction = self.get_direction()
        obs = [[x, y, self.get_speed(), np.sin(direction), np.cos(direction),
                self._convert_trajectory(self.get_trajectory()), 1]]
        for message in messages:
            c = message.contents
            obs.append([
                x - c["position"][0], y - c["position"][1], c["speed"], np.sin(c["direction"]), np.cos(c["direction"]),
                self._convert_trajectory(c["trajectory"]), 1
            ])
        for _ in range(6 - len(obs)):
            obs.append([0, 0, 0, 0, 0, 0, 0])

        self.last_obs = obs
        return np.array(obs)

    def get_reward(self) -> float:
        """Return's this vehicle's reward

        :return: The reward for this agent at this time step
        """
        surrounding_distances = [np.linalg.norm(np.array([obs[0], obs[1]])) for obs in self.last_obs[1:] if obs[6] == 1]
        # avg_distance = np.average(surrounding_distances)
        closest_distance = min(surrounding_distances) if len(surrounding_distances) > 0 else 100

        speed_component = 10 - np.abs(self.get_speed_limit() - self.get_speed())
        distance_component = self._distance_reward(closest_distance)
        return speed_component + distance_component

    def _convert_trajectory(self, trajectory):
        return self.trajectory_list.index(trajectory)

    @staticmethod
    def _distance_reward(distance):
        distance = max(distance, 0.001)
        return 5 * np.log(0.1 * distance)
