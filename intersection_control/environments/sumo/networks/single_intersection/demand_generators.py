from random import random
from typing import Dict, Tuple, List
from intersection_control.environments.sumo.sumo_environment import DemandGenerator, NewVehicleParams


class RandomDemandGenerator(DemandGenerator):
    """Generates vehicles with a Bernoulli distribution in different routes

    Will spawn vehicles with a probability, and on a route, provided to this
    DemandGenerator's constructor.
    """

    def __init__(self, rates: Dict[str, float], time_step_length: float):
        """Construct a RandomDemandGenerator

        :param Dict[str, float] rates: A dictionary mapping route IDs to the
            average vehicles per minute that should be produced on that route
        :param float time_step_length: The length of a single time step in
            the sumo simulation
        """
        self.time_step_length = time_step_length

        # convert rates from vehicles per minute to a probability of a spawn event occurring at each time step
        self.spawn_probabilities = {route: rate * self.time_step_length / 60 for route, rate in rates.items()}
        self.current_id = 0

    def step(self) -> List[NewVehicleParams]:
        return [NewVehicleParams(self.get_next_id(), route, depart_speed=10) for route, prob in
                self.spawn_probabilities.items() if random() <= prob]

    def get_next_id(self) -> str:
        result = self.current_id
        self.current_id += 1
        return str(result)


class ConflictingDemandGenerator(DemandGenerator):
    """Will spawn vehicles on different routes at times that should cause
    them to conflict when they arrive at the intersection

    Can be used to make sure the intersection control algorithm resolves
    this conflict.
    """

    def __init__(self):
        self.current_id = 0
        self.step_count = 0

    def step(self) -> List[NewVehicleParams]:
        self.step_count += 1
        if self.step_count % 200 != 0:
            return []

        next_id = self.get_next_id()
        conflicting = [("EW", 9), ("NW", 9.5), ("SW", 10)]

        return [NewVehicleParams(f"{route}{next_id}", route, depart_speed=depart_speed) for route, depart_speed in
                conflicting]

    def get_next_id(self) -> str:
        result = self.current_id
        self.current_id += 1
        return str(result)
