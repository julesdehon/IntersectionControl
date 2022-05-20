#!/usr/bin/env python
import logging

from intersection_control.algorithms.stip.stip_vehicle import StipVehicle
from intersection_control.environments.sumo import SumoEnvironment, RandomDemandGenerator
from intersection_control.algorithms.qb_im import QBIMIntersectionManager
from intersection_control.algorithms.qb_im import QBIMVehicle
from intersection_control.environments.sumo.networks.single_intersection.demand_generators import \
    ConflictingDemandGenerator

STEP_COUNT = 360000  # 1 Hour


"""
Performs an experiment using the QBIM vehicle and intersection manager in a
SUMO simulation environment.

Note the QBIM implementation is far from perfect and there are a few bugs
that will likely cause the simulation to crash at a certain point. These
will eventually have to be ironed out.
"""


def main():
    logging.basicConfig(level=logging.DEBUG)

    # demand_generator = RandomDemandGenerator({
    #     "NE": 2, "NS": 2, "NW": 2, "EN": 2, "ES": 2, "EW": 2, "SN": 2, "SE": 2, "SW": 2, "WN": 2, "WE": 2, "WS": 2
    # }, 0.05)
    demand_generator = ConflictingDemandGenerator()
    env = SumoEnvironment("intersection_control/environments/sumo/networks/single_intersection/intersection.sumocfg",
                          demand_generator=demand_generator, time_step=0.05, gui=True)

    intersection_managers = {QBIMIntersectionManager(intersection_id, env, 30, 0.05) for intersection_id in
                             env.intersections.get_ids()}
    vehicles = {QBIMVehicle(vehicle_id, env, communication_range=75) for vehicle_id in env.vehicles.get_ids()}

    for _ in range(STEP_COUNT):
        env.step()
        removed_vehicles = {v for v in vehicles if v.get_id() in env.get_removed_vehicles()}
        for v in removed_vehicles:
            v.destroy()
        new_vehicles = {QBIMVehicle(vehicle_id, env, communication_range=75) for vehicle_id in env.get_added_vehicles()}
        vehicles = (vehicles - removed_vehicles).union(new_vehicles)
        for vehicle in vehicles:
            vehicle.step()
        for intersection_manager in intersection_managers:
            intersection_manager.step()


# this is the main entry point of this script
if __name__ == "__main__":
    main()
