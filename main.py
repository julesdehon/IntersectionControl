#!/usr/bin/env python
import logging
from optparse import OptionParser

from VehicleProducer import RandomVehicleProducer
from intersection_control.environments.sumo import SumoEnvironment
from intersection_control.algorithms.qb_im import QBIMIntersectionManager
from intersection_control.algorithms.qb_im import QBIMVehicle
from intersection_control.environments.sumo.networks.single_intersection.demand_generators import RandomDemandGenerator, \
    ConflictingDemandGenerator

DEFAULT_STEP_COUNT = 360000  # 1 Hour


def get_options():
    opt_parser = OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False,
                          help="run the commandline version of sumo")
    opt_parser.add_option("-s", "--step-count", dest="step_count", default=DEFAULT_STEP_COUNT, type="int",
                          help="set the number of steps the simulation should run for")
    opt_parser.add_option("-a", "--algorithm", dest="algorithm", default="qb-im", type="choice",
                          choices=["qb-im", "ab-im", "decentralised"])
    options, args = opt_parser.parse_args()
    return options


# def im_for_algo(algorithm: str, environment: Environment) -> IntersectionManager:
#     if algorithm == "qb-im":
#         return QBIMIntersectionManager(im_sim_interface, 10, 0.05)
#     elif algorithm == "ab-im":
#         raise NotImplementedError
#     elif algorithm == "decentralised":
#         raise NotImplementedError
#     else:
#         raise ValueError


def main():
    logging.basicConfig(level=logging.DEBUG)
    options = get_options()

    demand_generator = RandomDemandGenerator({
        "NE": 2, "NS": 2, "NW": 2, "EN": 2, "ES": 2, "EW": 2, "SN": 2, "SE": 2, "SW": 2, "WN": 2, "WE": 2, "WS": 2
    }, 0.05)
    env = SumoEnvironment("intersection_control/environments/sumo/networks/single_intersection/intersection.sumocfg",
                          demand_generator=demand_generator, time_step=0.05)

    intersection_managers = [QBIMIntersectionManager(intersection_id, env, 10, 0.05) for intersection_id in
                             env.intersections.get_ids()]
    vehicles = [QBIMVehicle(vehicle_id, intersection_managers[0], env, communication_range=75) for vehicle_id in
                env.vehicles.get_ids()]

    for _ in range(options.step_count):
        env.step()
        vehicles[:] = [v for v in vehicles if v.get_id() not in env.get_removed_vehicles()] + [
            QBIMVehicle(vehicle_id, intersection_managers[0], env, communication_range=75) for vehicle_id in
            env.get_added_vehicles()]
        for vehicle in vehicles:
            vehicle.step()
        for intersection_manager in intersection_managers:
            intersection_manager.step()


# this is the main entry point of this script
if __name__ == "__main__":
    main()
