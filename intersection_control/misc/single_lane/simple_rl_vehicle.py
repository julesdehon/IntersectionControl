from intersection_control.core import Vehicle, Environment


class SimpleRLVehicle(Vehicle):
    def __init__(self, vehicle_id: str, environment: Environment):
        super().__init__(vehicle_id, environment)
        self.last_speed = self.get_speed()

    def step(self):
        """Perform a single step in the simulation

        In this experiment I don't make use of this, as the vehicle never
        makes decisions for itself, but is instead controlled by the
        apply_action method
        """
        pass

    def apply_action(self, action):
        """Applies the given action to the vehicle

        Possible actions:
            0: NOOP
            1: ACCELERATE
            2: DECELERATE

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

        The observation is simply the vehicle's current speed

        :return: The observation for this agent at this time step
        """
        speed = self.get_speed()
        if speed < 0 or speed > 500:
            return [0]  # Fixes weird SUMO bug where speed was massive
        return [speed]

    def get_reward(self) -> float:
        """Return's this vehicle's reward

        The reward is simply the change in speed since the last time
        this function was called

        :return: The reward for this agent at this time step
        """
        speed_change = self.get_speed() - self.last_speed
        if speed_change < -100 or speed_change > 100:
            speed_change = 0  # Fixes weird SUMO bug giving huge negative speeds
        self.last_speed = self.get_speed()
        return speed_change
