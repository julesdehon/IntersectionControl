#!/usr/bin/env python
import os
import sys
import logging

# we need to import python modules from the $SUMO_HOME/tools directory
from VehicleProducer import RandomVehicleProducer, ConflictingVehicleProducer

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")
import sumolib
import traci
from optparse import OptionParser
from IMSimulationInterface import IMSimulationInterface
from VehicleSimulationInterface import VehicleSimulationInterface
from intersection_control.interfaces import IntersectionManager, Vehicle
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


def im_for_algo(algorithm: str, im_sim_interface: IMSimulationInterface) -> IntersectionManager:
    if algorithm == "qb-im":
        return QBIMIntersectionManager(im_sim_interface, 10, 0.05)
    elif algorithm == "ab-im":
        raise NotImplementedError
    elif algorithm == "decentralised":
        raise NotImplementedError
    else:
        raise ValueError


def main():
    logging.basicConfig(level=logging.DEBUG)
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    sumo_binary = sumolib.checkBinary('sumo') if options.nogui else sumolib.checkBinary('sumo-gui')
    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumo_binary, "-c", "network/intersection.sumocfg", "--step-length", "0.05",
                 "--collision.check-junctions", "--default.speeddev", "0"])
    traci.simulationStep()  # Perform a single step so all vehicles are loaded into the network.

    im_simulation_interface = IMSimulationInterface("intersection")
    intersection_manager = im_for_algo(options.algorithm, im_simulation_interface)

    vehicles = []
    rates = {
        "N": (10, {"NE": 1/3, "NS": 1/3, "NW": 1/3}),
        "E": (10, {"EN": 1/3, "ES": 1/3, "EW": 1/3}),
        "S": (10, {"SN": 1 / 3, "SE": 1 / 3, "SW": 1 / 3}),
        "W": (10, {"WN": 1 / 3, "WE": 1 / 3, "WS": 1 / 3})
    }
    vehicle_producer = RandomVehicleProducer(vehicles, rates, options.algorithm, intersection_manager,
                                             im_simulation_interface)
    # vehicle_producer = ConflictingVehicleProducer(vehicles, options.algorithm, intersection_manager,
    #                                               im_simulation_interface)

    step = 0
    while step < options.step_count:
        traci.simulationStep()
        vehicle_producer.step()
        for vehicle in vehicles:
            vehicle.step()
        intersection_manager.step()
        step += 1

    traci.close(False)


# this is the main entry point of this script
if __name__ == "__main__":
    main()
