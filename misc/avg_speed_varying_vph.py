from typing import Optional, Callable
import numpy as np
from intersection_control.core import Vehicle, Environment, IntersectionManager
from intersection_control.environments import SumoEnvironment
from intersection_control.algorithms import qb_im, stip
from intersection_control.core.performance_indication import MetricCollector, Metric
from intersection_control.environments.sumo import RandomDemandGenerator

TIME_STEP = 0.05
VPMs = [1, 2, 3, 4, 5]
RUNS_PER_VPM = 10
STEPS_PER_RUN = int((5 * 60) / TIME_STEP)  # 5 minutes
VEHICLE_FACTORIES = {
    "stip": lambda vid, env: stip.STIPVehicle(vid, env),
    "qb_im": lambda vid, env: qb_im.QBIMVehicle(vid, env, communication_range=50)
}
IM_FACTORIES = {
    "stip": None,
    "qb_im": lambda imid, env: qb_im.QBIMIntersectionManager(imid, env, 10, TIME_STEP)
}


def main():
    for vpm in VPMs:
        for algo, v_factory in VEHICLE_FACTORIES.items():
            total_speeds = 0
            for _ in range(RUNS_PER_VPM):
                total_speeds += run_experiment(vpm, v_factory, IM_FACTORIES[algo])
            avg_speed = total_speeds / RUNS_PER_VPM
            print(f"[{vpm}] [{algo}] {avg_speed}")


def run_experiment(vpm: float, v_factory: Callable[[str, Environment], Vehicle],
                   im_factory: Optional[Callable[[str, Environment], IntersectionManager]]):
    demand_generator = RandomDemandGenerator({
        route: vpm for route in ["NE", "NS", "NW", "EN", "ES", "EW", "SN", "SE", "SW", "WN", "WE", "WS"]
    }, TIME_STEP)
    env = SumoEnvironment(
        "../intersection_control/environments/sumo/networks/single_intersection/intersection.sumocfg",
        demand_generator=demand_generator, time_step=TIME_STEP, gui=False)
    vehicles = {v_factory(vehicle_id, env) for vehicle_id in env.vehicles.get_ids()}
    intersection_managers = {im_factory(intersection_id, env) for intersection_id in
                             env.intersections.get_ids()} if im_factory else None

    metric_collector = MetricCollector(env, Metric.AVG_SPEED)

    for _ in range(STEPS_PER_RUN):
        env.step()
        removed_vehicles = {v for v in vehicles if v.get_id() in env.get_removed_vehicles()}
        for v in removed_vehicles:
            v.destroy()
        new_vehicles = {v_factory(vehicle_id, env) for vehicle_id in env.get_added_vehicles()}
        vehicles = (vehicles - removed_vehicles).union(new_vehicles)
        for vehicle in vehicles:
            vehicle.step()
        if intersection_managers:
            for intersection_manager in intersection_managers:
                intersection_manager.step()
        metric_collector.poll()

    for v in vehicles:
        v.destroy()

    env.close()

    return np.mean([speed for speed in metric_collector.get_results()[Metric.AVG_SPEED] if speed is not None])


if __name__ == '__main__':
    main()
