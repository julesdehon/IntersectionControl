from __future__ import annotations
import sys
import os
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Union, Optional

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
                 time_step: float = 0.05, gui: bool = True):
        net_file = next(sumolib.xml.parse_fast(net_config_file, "net-file", "value")).value
        route_file = next(sumolib.xml.parse_fast(net_config_file, "route-files", "value")).value
        self._intersections = SumoIntersectionHandler(net_file, route_file)
        self._vehicles = SumoVehicleHandler(net_file)
        self.demand_generator = demand_generator

        # this script has been called from the command line. It will start sumo as a
        # server, then connect and run
        sumo_binary = sumolib.checkBinary('sumo-gui') if gui else sumolib.checkBinary('sumo')
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        traci.start([sumo_binary, "-c", net_config_file, "--step-length", str(time_step),
                     "--collision.check-junctions", "--default.speeddev", "0"])
        traci.simulationStep()  # Perform a single step so all vehicles are loaded into the network.

    @property
    def intersections(self) -> IntersectionHandler:
        return self._intersections

    @property
    def vehicles(self) -> VehicleHandler:
        return self._vehicles

    def get_current_time(self) -> float:
        return traci.simulation.getTime()

    def step(self):
        traci.simulationStep()
        if self.demand_generator is not None:
            for v in self.demand_generator.step():
                traci.vehicle.add(v.vehID, v.routeID, departSpeed=v.depart_speed)

    def get_removed_vehicles(self) -> List[str]:
        return traci.simulation.getArrivedIDList()

    def get_added_vehicles(self) -> List[str]:
        return traci.simulation.getDepartedIDList()

    def clear(self):
        for v in traci.vehicle.getIDList():
            traci.vehicle.remove(v)


@dataclass
class NewVehicleParams:
    """Used by the SUMO environment to add a new vehicle to the environment

    This contains all the necessary information required for the SUMO environment
    to spawn a new vehicle at a particular location - namely the vehicle's id,
    route and departure speed
    """
    vehID: str
    routeID: str
    depart_speed: Union[float, str] = 0


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
