#!/usr/bin/env python
import os
import sys
# we need to import python modules from the $SUMO_HOME/tools directory
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
        return QBIMIntersectionManager(im_sim_interface)
    elif algorithm == "ab-im":
        raise NotImplementedError
    elif algorithm == "decentralised":
        raise NotImplementedError
    else:
        raise ValueError


def vehicle_for_algo(algorithm: str, im: IntersectionManager, veh_sim_interface: VehicleSimulationInterface) -> Vehicle:
    if algorithm == "qb-im":
        return QBIMVehicle(im, veh_sim_interface, communication_range=75)
    elif algorithm == "ab-im":
        raise NotImplementedError
    elif algorithm == "decentralised":
        raise NotImplementedError
    else:
        raise ValueError


def main():
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    sumo_binary = sumolib.checkBinary('sumo') if options.nogui else sumolib.checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumo_binary, "-c", "network/intersection.sumocfg", "--step-length", "0.25"])
    traci.simulationStep()  # Perform a single step so all vehicles are loaded into the network.

    im_simulation_interface = IMSimulationInterface("intersection")
    intersection_manager = im_for_algo(options.algorithm, im_simulation_interface)

    vehicle_simulation_interfaces = [VehicleSimulationInterface(vehicle_id, im_simulation_interface) for vehicle_id in
                                     traci.vehicle.getIDList()]
    vehicles = [vehicle_for_algo(options.algorithm, intersection_manager, vehicle_simulation_interface) for
                vehicle_simulation_interface in vehicle_simulation_interfaces]

    step = 0
    while step < options.step_count:
        traci.simulationStep()
        for vehicle in vehicles:
            vehicle.step()
        intersection_manager.step()
        step += 1

    traci.close(False)


# this is the main entry point of this script
if __name__ == "__main__":
    main()
