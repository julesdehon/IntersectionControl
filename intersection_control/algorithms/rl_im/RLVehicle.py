from intersection_control.core import Vehicle, Environment
import math


class RLVehicle(Vehicle):
    def __init__(self, vehicle_id: str, environment: Environment):
        super().__init__(vehicle_id, environment)
        self.last_position = self.get_position()

    def step(self):
        """Perform a single step in the simulation

        This method should probably not be called when training - as it
        does not take an action externally. Once a policy has been trained,
        and we use this class in "evaluate" mode (rather than "train" mode),
        this method should use the policy to generate an action based on its
        observations, and apply that action.
        """
        pass

    def apply_action(self, action):
        """Applies the given action to the vehicle

        TODO: Select an action space
        This method will have to interact with the environment in order
        to perform the given action

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

        TODO: Select an observation space

        :return: The observation for this agent at this time step
        """
        return [self.get_speed()]

    def get_reward(self) -> float:
        """Return's this vehicle's reward

        TODO: Formulate the reward function

        :return: The reward for this agent at this time step
        """
        x, y = self.get_position()
        x1, y1 = self.last_position
        distance_moved = math.sqrt((x - x1) ** 2 + (y - y1) ** 2)
        self.last_position = (x, y)
        # print(f"distance moved: {distance_moved}")
        return distance_moved
