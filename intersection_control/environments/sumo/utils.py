from abc import ABC, abstractmethod
from dataclasses import dataclass
import random
from typing import Union, List, Dict

random.seed(42)


class ControlType:
    """Defines the level of control we have over a vehicle in SUMO

    Will affect the behaviour of the set_desired_speed method

    :cvar int MANUAL: The vehicle will exactly follow the speed provided to it
        using the set_desired_speed method - this may lead to collisions if this
        causes the vehicle to catch up with the vehicle ahead and the speed is not
        adjusted manually to prevent the collision
    :cvar int WITH_SAFETY_PRECAUTIONS: The vehicle will attempt to reach the desired
        speed, but will slow down to avoid collisions with other vehicles. This can
        be thought of as driving with a collision detection and avoidance system
        in an autonomous vehicle.
    """
    MANUAL = 0
    WITH_SAFETY_PRECAUTIONS = 1


@dataclass
class NewVehicleParams:
    """Used by the SUMO environment to add a new vehicle to the environment

    This contains all the necessary information required for the SUMO environment
    to spawn a new vehicle at a particular location - namely the vehicle's id,
    route and departure speed

    :ivar str veh_id: The ID of the vehicle that should be spawned
    :ivar str route_id: The ID of the route that the vehicle should be spawned on
    :ivar Union[float, str] depart_speed: The initial speed the vehicle should have when it is spawned
    :ivar float depart_pos: The distance along the trajectory the vehicle should be spawned at
    :ivar ControlType control_type: The level of control the set_desired_speed method will have on
        the given vehicle
    """
    veh_id: str
    route_id: str
    depart_speed: Union[float, str] = 0
    depart_pos: float = 0
    control_type: ControlType = ControlType.WITH_SAFETY_PRECAUTIONS


class DemandGenerator(ABC):
    """Abstract class to programmatically add new vehicles to the environment

    This should be subclassed to provide different demand generation implementations.
    There is an example in environments/sumo/networks/single_intersection/demand_generators.

    At every time step, the SumoEnvironment will call :func:`step`, which should return
    a list of :class:`NewVehicleParams` - one for each new vehicle that should be added
    to the simulation in this time step. :class:`NewVehicleParams` provides all the
    information necessary to spawn a new vehicle - namely its id, route and departure
    speed
    """

    @abstractmethod
    def step(self) -> List[NewVehicleParams]:
        """Method called at every time step by :class:`SumoEnvironment`

        :return: List of vehicles that should be added to the environment
            in this simulation step
        """
        raise NotImplementedError


class ScenarioGenerator(DemandGenerator):
    """Demand generator that produces vehicles at predefined locations
    either on the very first call to :func:`step`, or the call immediately
    following a call to :func:`reset`.
    """

    def __init__(self, new_vehicle_params: List[NewVehicleParams]):
        """Construct a RandomDemandGenerator

        :param List[NewVehicleParams] new_vehicle_params: A list of NewVehicleParams
            describing the departure parameters of the vehicles that are spawned
            initially and after each reset
        """
        self.should_spawn = True
        self.new_vehicle_params = new_vehicle_params.copy()

    def step(self) -> List[NewVehicleParams]:
        if not self.should_spawn:
            return []

        self.should_spawn = False
        return self.new_vehicle_params.copy()

    def reset(self):
        self.should_spawn = True


class RandomDemandGenerator(DemandGenerator):
    """Generates vehicles with a Bernoulli distribution in different routes

    Will spawn vehicles with a probability, and on a route, provided to this
    DemandGenerator's constructor.
    """

    def __init__(self, rates: Dict[str, float], time_step_length: float, depart_speed: float = 10,
                 control_type: ControlType = ControlType.WITH_SAFETY_PRECAUTIONS):
        """Construct a RandomDemandGenerator

        :param Dict[str, float] rates: A dictionary mapping route IDs to the
            average vehicles per minute that should be produced on that route
        :param float time_step_length: The length of a single time step in
            the sumo simulation
        """
        self.time_step_length = time_step_length
        self.depart_speed = depart_speed
        self.control_type = control_type

        # convert rates from vehicles per minute to a probability of a spawn event occurring at each time step
        self.spawn_probabilities = {route: rate * self.time_step_length / 60 for route, rate in rates.items()}
        self.current_id = 0

    def step(self) -> List[NewVehicleParams]:
        return [
            NewVehicleParams(self.get_next_id(), route, depart_speed=self.depart_speed, control_type=self.control_type)
            for route, prob in self.spawn_probabilities.items() if random.random() <= prob
        ]

    def get_next_id(self) -> str:
        result = self.current_id
        self.current_id += 1
        return str(result)
