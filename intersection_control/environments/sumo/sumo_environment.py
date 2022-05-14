from __future__ import annotations
import sys
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from random import random
from typing import List, Union, Optional, Dict

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import sumolib
import traci

from intersection_control.core import Environment
from intersection_control.core.environment import VehicleHandler, IntersectionHandler
from .sumo_intersection_handler import SumoIntersectionHandler
from .sumo_vehicle_handler import SumoVehicleHandler


class SumoEnvironment(Environment):
    def __init__(self, net_config_file: str, demand_generator: Optional[DemandGenerator] = None,
                 time_step: float = 0.05, gui: bool = True, warnings=True):
        net_file = os.path.join(os.path.dirname(net_config_file),
                                next(sumolib.xml.parse_fast(net_config_file, "net-file", "value")).value)
        route_file = os.path.join(os.path.dirname(net_config_file),
                                  next(sumolib.xml.parse_fast(net_config_file, "route-files", "value")).value)
        self._intersections = SumoIntersectionHandler(net_file, route_file)
        self._vehicles = SumoVehicleHandler(net_file)
        self.demand_generator = demand_generator

        # this script has been called from the command line. It will start sumo as a
        # server, then connect and run
        sumo_binary = sumolib.checkBinary('sumo-gui') if gui else sumolib.checkBinary('sumo')
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        sumo_cmd = [
            sumo_binary,
            "-c", net_config_file,
            "--step-length", str(time_step),
            "--collision.check-junctions",
            "--default.speeddev", "0",
            "--collision.action", "remove"
        ]
        if not warnings:
            sumo_cmd.append("--no-warnings")
        traci.start(sumo_cmd)
        traci.simulationStep()  # Perform a single step so all vehicles are loaded into the network.

    @staticmethod
    def close():
        traci.close(False)

    @property
    def intersections(self) -> IntersectionHandler:
        return self._intersections

    @property
    def vehicles(self) -> VehicleHandler:
        return self._vehicles

    def get_current_time(self) -> float:
        return traci.simulation.getTime()

    def step(self):
        if self.demand_generator is not None:
            for v in self.demand_generator.step():
                traci.vehicle.add(v.veh_id, v.route_id, departSpeed=v.depart_speed, departPos=v.depart_pos)
                traci.vehicle.setColor(v.veh_id, [255, 255, 255, 255])
                if v.control_type == ControlType.MANUAL:
                    traci.vehicle.setSpeedMode(v.veh_id, 0b100110)
                    traci.vehicle.setSpeed(v.veh_id, v.depart_speed)  # The vehicle won't accelerate to the road's limit
        traci.simulationStep()

    def get_removed_vehicles(self) -> List[str]:
        return traci.simulation.getArrivedIDList()

    def get_added_vehicles(self) -> List[str]:
        return traci.simulation.getDepartedIDList()

    def clear(self):
        for v in traci.vehicle.getIDList():
            traci.vehicle.remove(v)
        traci.simulation.clearPending()
        for _ in range(10):
            traci.simulationStep()  # Sometimes takes a few tries to flush them out


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
