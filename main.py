#!/usr/bin/env python
import logging

# we need to import python modules from the $SUMO_HOME/tools directory
from VehicleProducer import RandomVehicleProducer

from optparse import OptionParser
from intersection_control.core import Vehicle, Environment
from intersection_control.core import IntersectionManager
from intersection_control.environments.sumo import SumoEnvironment
from intersection_control.qb_im.QBIMIntersectionManager import QBIMIntersectionManager
from intersection_control.qb_im.QBIMVehicle import QBIMVehicle

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

    env = SumoEnvironment()

    intersection_managers = [QBIMIntersectionManager(intersection_id, env, 10, 0.05) for intersection_id in env.intersections.get_ids()]
    vehicles = [QBIMVehicle(vehicle_id, intersection_managers[0], env, communication_range=75) for vehicle_id in env.vehicles.get_ids()]

    rates = {
        "N": (5, {"NE": 1/3, "NS": 1/3, "NW": 1/3}),
        "E": (5, {"EN": 1/3, "ES": 1/3, "EW": 1/3}),
        "S": (5, {"SN": 1 / 3, "SE": 1 / 3, "SW": 1 / 3}),
        "W": (5, {"WN": 1 / 3, "WE": 1 / 3, "WS": 1 / 3})
    }
    vehicle_producer = RandomVehicleProducer(vehicles, rates, options.algorithm, intersection_managers[0], env)
    # vehicle_producer = ConflictingVehicleProducer(vehicles, options.algorithm, intersection_manager,
    #                                               im_simulation_interface)

    for _ in range(options.step_count):
        env.step()
        vehicle_producer.step()
        for vehicle in vehicles:
            vehicle.step()
        for intersection_manager in intersection_managers:
            intersection_manager.step()


# this is the main entry point of this script
if __name__ == "__main__":
    main()
