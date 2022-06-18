from __future__ import annotations
import sys
import os
import random
from typing import List, Optional

from .utils import DemandGenerator, ControlType

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import sumolib
import traci
import traci.constants as tc
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
        self.net = sumolib.net.readNet(net_file, withInternal=True)
        self.routes = {route.id: route.edges.split() for route in
                       sumolib.xml.parse_fast(route_file, 'route', ['id', 'edges'])}
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
            "--collision.action", "warn",
            "--no-step-log"
        ]
        if not warnings:
            sumo_cmd.append("--no-warnings")
        traci.start(sumo_cmd)
        self.subscription_junction_id = traci.junction.getIDList()[0]
        traci.junction.subscribeContext(self.subscription_junction_id, tc.CMD_GET_VEHICLE_VARIABLE, 100_000_000,
                                        [tc.VAR_SPEED, tc.VAR_POSITION, tc.VAR_ROAD_ID, tc.VAR_LANE_ID, tc.VAR_LENGTH,
                                         tc.VAR_WIDTH, tc.VAR_ROUTE_ID, tc.VAR_LANEPOSITION, tc.VAR_ANGLE,
                                         tc.VAR_ALLOWED_SPEED, tc.VAR_ACCELERATION, tc.VAR_ACCEL, tc.VAR_DECEL])
        traci.simulation.subscribe([tc.VAR_TIME, tc.VAR_ARRIVED_VEHICLES_IDS, tc.VAR_DEPARTED_VEHICLES_IDS,
                                    tc.VAR_COLLIDING_VEHICLES_IDS])
        traci.simulationStep()  # Perform a single step so all vehicles are loaded into the network.
        traci.junction.getContextSubscriptionResults(self.subscription_junction_id)
        self.subscription_results = traci.simulation.getSubscriptionResults()

        self._intersections = SumoIntersectionHandler(self.net, self.routes)
        self._vehicles = SumoVehicleHandler(self.net)

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
        return self.subscription_results[tc.VAR_TIME]

    def step(self):
        if self.demand_generator is not None:
            for v in self.demand_generator.step():
                lanes_for_route = [l.getIndex() for l in self.net.getEdge(self.routes[v.route_id][0]).getLanes() if
                                   self.routes[v.route_id][1] in [conn.getTo().getID() for conn in l.getOutgoing()]]
                traci.vehicle.add(v.veh_id, v.route_id, departLane=random.choice(lanes_for_route),
                                  departSpeed=v.depart_speed, departPos=v.depart_pos)
                traci.vehicle.setColor(v.veh_id, [255, 255, 255, 255])
                if v.control_type == ControlType.MANUAL:
                    traci.vehicle.setSpeedMode(v.veh_id, 0b100110)
                    traci.vehicle.setSpeed(v.veh_id, v.depart_speed)  # The vehicle won't accelerate to the road's limit
                traci.vehicle.setLaneChangeMode(v.veh_id, 0b010000000101)
        traci.simulationStep()
        self.vehicles.subscription_results = traci.junction.getContextSubscriptionResults(self.subscription_junction_id)
        self.subscription_results = traci.simulation.getSubscriptionResults()

    def get_removed_vehicles(self) -> List[str]:
        return list(set(self.subscription_results[tc.VAR_ARRIVED_VEHICLES_IDS]
                        + self.subscription_results[tc.VAR_COLLIDING_VEHICLES_IDS]))

    def get_added_vehicles(self) -> List[str]:
        return self.subscription_results[tc.VAR_DEPARTED_VEHICLES_IDS]

    def clear(self):
        for v in traci.vehicle.getIDList():
            traci.vehicle.remove(v)
        traci.simulation.clearPending()
        for _ in range(10):
            traci.simulationStep()  # Sometimes takes a few tries to flush them out
