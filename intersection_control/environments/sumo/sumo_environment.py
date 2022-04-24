import sys
import os
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import sumolib
import traci

from intersection_control.core import Environment
from intersection_control.core.environment import VehicleHandler, IntersectionHandler
from intersection_control.environments.sumo import SumoIntersectionHandler, SumoVehicleHandler


class SumoEnvironment(Environment):
    def __init__(self):
        self._intersections = SumoIntersectionHandler()
        self._vehicles = SumoVehicleHandler()

        # this script has been called from the command line. It will start sumo as a
        # server, then connect and run
        sumo_binary = sumolib.checkBinary('sumo-gui')
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        traci.start([sumo_binary, "-c", "network/intersection.sumocfg", "--step-length", "0.05",
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
