#!/usr/bin/env python
import os
import sys
from optparse import OptionParser

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

DEFAULT_STEP_COUNT = 360000  # 1 Hour


def get_options():
    opt_parser = OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False,
                          help="run the commandline version of sumo")
    opt_parser.add_option("-s", "--step-count", dest="step_count", default=DEFAULT_STEP_COUNT, type="int",
                          help="set the number of steps the simulation should run for")
    options, args = opt_parser.parse_args()
    return options


def main():
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    sumo_binary = checkBinary('sumo') if options.nogui else checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumo_binary, "-c", "network/intersection.sumocfg"])

    step = 0
    while step < options.step_count:
        traci.simulationStep()
        step += 1

    traci.close()


# this is the main entry point of this script
if __name__ == "__main__":
    main()
