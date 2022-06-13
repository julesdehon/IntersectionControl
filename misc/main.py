#!/usr/bin/env python
import logging

from intersection_control.algorithms.stip.stip_vehicle import STIPVehicle
from intersection_control.algorithms.traffic_light.tl_intersection_manager import TLIntersectionManager
from intersection_control.algorithms.traffic_light.tl_vehicle import TLVehicle
from intersection_control.environments.sumo import SumoEnvironment, RandomDemandGenerator
from intersection_control.algorithms.qb_im import QBIMIntersectionManager, QBIMVehicle
from intersection_control.environments.sumo.networks.single_intersection.demand_generators import \
    ConflictingDemandGenerator

STEP_COUNT = 360000  # 1 Hour
TL_PHASES = [
    # ((Green, Yellow, Red), Duration)
    (({"NE", "NS", "NW"}, set(), {"EN", "ES", "EW", "SN", "SE", "SW", "WN", "WE", "WS"}), 10),
    ((set(), {"NE", "NS", "NW"}, {"EN", "ES", "EW", "SN", "SE", "SW", "WN", "WE", "WS"}), 2),
    (({"EN", "ES", "EW"}, set(), {"NE", "NS", "NW", "SN", "SE", "SW", "WN", "WE", "WS"}), 10),
    ((set(), {"EN", "ES", "EW"}, {"NE", "NS", "NW", "SN", "SE", "SW", "WN", "WE", "WS"}), 2),
    (({"SN", "SE", "SW"}, set(), {"NE", "NS", "NW", "EN", "ES", "EW", "WN", "WE", "WS"}), 10),
    ((set(), {"SN", "SE", "SW"}, {"NE", "NS", "NW", "EN", "ES", "EW", "WN", "WE", "WS"}), 2),
    (({"WN", "WE", "WS"}, set(), {"NE", "NS", "NW", "EN", "ES", "EW", "SN", "SE", "SW"}), 10),
    ((set(), {"WN", "WE", "WS"}, {"NE", "NS", "NW", "EN", "ES", "EW", "SN", "SE", "SW"}), 2)
]

im_factory = {
    "qb_im": lambda imid, env: QBIMIntersectionManager(imid, env, 30, 0.05),
    "stip": None,
    "tl": lambda imid, env: TLIntersectionManager(imid, env, TL_PHASES)
}

vehicle_factory = {
    "qb_im": lambda vid, env: QBIMVehicle(vid, env, communication_range=75),
    "stip": lambda vid, env: STIPVehicle(vid, env),
    "tl": lambda vid, env: TLVehicle(vid, env)
}


def main():
    logging.basicConfig(level=logging.DEBUG)
    algo = "qb_im"
    rate = 2

    # demand_generator = RandomDemandGenerator({
    #     "NE": rate, "NS": rate, "NW": rate,
    #     "EN": rate, "ES": rate, "EW": rate,
    #     "SN": rate, "SE": rate, "SW": rate,
    #     "WN": rate, "WE": rate, "WS": rate
    # }, 0.05)
    demand_generator = ConflictingDemandGenerator()
    env = SumoEnvironment("intersection_control/environments/sumo/networks/single_intersection/intersection.sumocfg",
                          demand_generator=demand_generator, time_step=0.05, gui=True)

    intersection_managers = {im_factory[algo](intersection_id, env) for intersection_id in
                             env.intersections.get_ids()} if im_factory[algo] else None
    vehicles = {vehicle_factory[algo](vehicle_id, env) for vehicle_id in env.vehicles.get_ids()}

    for _ in range(STEP_COUNT):
        env.step()
        removed_vehicles = {v for v in vehicles if v.get_id() in env.get_removed_vehicles()}
        for v in removed_vehicles:
            v.destroy()
        new_vehicles = {vehicle_factory[algo](vehicle_id, env) for vehicle_id in env.get_added_vehicles()}
        vehicles = (vehicles - removed_vehicles).union(new_vehicles)
        for vehicle in vehicles:
            vehicle.step()
        if intersection_managers:
            for im in intersection_managers:
                im.step()


# this is the main entry point of this script
if __name__ == "__main__":
    main()
