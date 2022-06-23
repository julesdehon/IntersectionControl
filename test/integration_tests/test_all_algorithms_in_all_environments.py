import unittest
import math
from os.path import join

from intersection_control.algorithms.qb_im import QBIMIntersectionManager, QBIMVehicle
from intersection_control.algorithms.stip import STIPVehicle
from intersection_control.algorithms.traffic_light import TLIntersectionManager, TLVehicle
from intersection_control.communication import DistanceBasedUnit
from intersection_control.environments import SumoEnvironment
from intersection_control.environments.sumo import RandomDemandGenerator
from misc.utils import SINGLE_INTERSECTION_TL_PHASES, ROOT_DIR

RATE = 1
TIME_STEP = 0.05
FIVE_MINUTES = math.floor((60 * 5) / TIME_STEP)

im_factories = [
    lambda imid, env: QBIMIntersectionManager(
        imid, env, 30, 0.05, DistanceBasedUnit(imid, 75, lambda: env.intersections.get_position(imid))),
    None,
    lambda imid, env: TLIntersectionManager(imid, env, SINGLE_INTERSECTION_TL_PHASES)
]

vehicle_factories = [
    lambda vid, env: QBIMVehicle(vid, env, DistanceBasedUnit(vid, 75, lambda: env.vehicles.get_position(vid))),
    lambda vid, env: STIPVehicle(vid, env, DistanceBasedUnit(vid, 125, lambda: env.vehicles.get_position(vid))),
    lambda vid, env: TLVehicle(vid, env)
]

environment_factories = [
    lambda: SumoEnvironment(
        join(ROOT_DIR, "intersection_control/environments/sumo/networks/single_intersection/intersection.sumocfg"),
        RandomDemandGenerator({
            "NE": RATE, "NS": RATE, "NW": RATE,
            "EN": RATE, "ES": RATE, "EW": RATE,
            "SN": RATE, "SE": RATE, "SW": RATE,
            "WN": RATE, "WE": RATE, "WS": RATE
        }, 0.05),
        0.05,
        False,
        False
    )
]


class TestAlgorithmInEnvironment(unittest.TestCase):
    def test_runs_for_at_least_5_minutes(self):
        for i, make_im in enumerate(im_factories):
            make_vehicle = vehicle_factories[i]
            for make_env in environment_factories:
                with self.subTest():
                    env = make_env()
                    if make_im:
                        ims = {make_im(imid, env) for imid in env.intersections.get_ids()}
                    vehicles = {make_vehicle(vid, env) for vid in env.vehicles.get_ids()}

                    for _ in range(FIVE_MINUTES):
                        env.step()
                        removed_vehicles = {v for v in vehicles if v.get_id() in env.get_removed_vehicles()}
                        for v in removed_vehicles:
                            v.destroy()
                        new_vehicles = {make_vehicle(vehicle_id, env) for vehicle_id in env.get_added_vehicles()}
                        vehicles = (vehicles - removed_vehicles).union(new_vehicles)
                        for vehicle in vehicles:
                            vehicle.step()
                        if make_im:
                            for im in ims:
                                im.step()
                    env.close()
