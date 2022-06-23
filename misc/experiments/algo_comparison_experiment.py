#!/usr/bin/env python
from typing import Optional, Callable, List, Set, Dict
import numpy as np
import time
from tqdm import tqdm
from intersection_control.algorithms.traffic_light import TLIntersectionManager, TLVehicle
from intersection_control.communication import DistanceBasedUnit
from intersection_control.core import Vehicle, Environment, IntersectionManager
from intersection_control.environments import SumoEnvironment
from intersection_control.algorithms import qb_im, stip
from intersection_control.core.performance_indication import MetricCollector, Metric
from intersection_control.environments.sumo import RandomDemandGenerator
from misc.utils import SINGLE_INTERSECTION_TL_PHASES, ROOT_DIR

TIME_STEP = 0.05
VPMs = np.arange(0.2, 1.51, 0.05)
RUNS_PER_VPM = 3
STEPS_PER_RUN = int((20 * 60) / TIME_STEP)  # 20 minutes
VEHICLE_FACTORIES = {
    "stip": lambda vid, env: stip.STIPVehicle(vid, env,
                                              DistanceBasedUnit(vid, 125, lambda: env.vehicles.get_position(vid))),
    "qb_im": lambda vid, env: qb_im.QBIMVehicle(vid, env,
                                                DistanceBasedUnit(vid, 75, lambda: env.vehicles.get_position(vid))),
    "tl": lambda vid, env: TLVehicle(vid, env)
}
IM_FACTORIES = {
    "stip": None,
    "qb_im": lambda imid, env: qb_im.QBIMIntersectionManager(
        imid, env, 30, TIME_STEP, DistanceBasedUnit(imid, 75, lambda: env.intersections.get_position(imid))),
    "tl": lambda imid, env: TLIntersectionManager(imid, env, SINGLE_INTERSECTION_TL_PHASES)
}
METRICS_TO_COLLECT = [Metric.TIME, Metric.ALL_VEHICLE_IDS, Metric.MESSAGES_EXCHANGED, Metric.WALL_TIME]


def main():
    f = open(f"{ROOT_DIR}/misc/experiments/out/{time.time()}-algo_comparison_experiment.csv", "w")
    f.write("vpm,algo,delay,messages_exchanged,time_per_step\n")
    for vpm in VPMs:
        print(f"Running experiments for {vpm} vehicles per minute")
        for algo, v_factory in VEHICLE_FACTORIES.items():
            print(f"Running experiments for {algo} algorithm")
            totals = np.array([0.] * 3)
            successful_experiments = 0
            for i in range(RUNS_PER_VPM):
                print(f"Run {i + 1}/{RUNS_PER_VPM}")
                results = run_experiment(vpm, v_factory, IM_FACTORIES[algo])
                if results:
                    totals += results
                    successful_experiments += 1
            avgs = totals / successful_experiments
            f.write(f"{vpm},{algo},{','.join([str(avg) for avg in avgs])}\n")
            f.flush()
    f.close()


def run_experiment(vpm: float, v_factory: Callable[[str, Environment], Vehicle],
                   im_factory: Optional[Callable[[str, Environment], IntersectionManager]]):
    experiment_start = time.time()
    demand_generator = RandomDemandGenerator({
        route: vpm for route in ["NE", "NS", "NW", "EN", "ES", "EW", "SN", "SE", "SW", "WN", "WE", "WS"]
    }, TIME_STEP)
    env = SumoEnvironment(
        f"{ROOT_DIR}/intersection_control/environments/sumo/networks/single_intersection/intersection.sumocfg",
        demand_generator=demand_generator, time_step=TIME_STEP, gui=False)
    vehicles = {v_factory(vehicle_id, env) for vehicle_id in env.vehicles.get_ids()}
    intersection_managers = {im_factory(intersection_id, env) for intersection_id in
                             env.intersections.get_ids()} if im_factory else None

    metric_collector = MetricCollector(env, *METRICS_TO_COLLECT, messaging_unit_class=DistanceBasedUnit)

    for _ in tqdm(range(STEPS_PER_RUN)):
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

        if time.time() - experiment_start > 60 * 10:
            # More than 10 mins for experiment
            print("Experiment took too long, continuing to the next one")
            for v in vehicles:
                v.destroy()
            env.close()
            return None

    for v in vehicles:
        v.destroy()

    env.close()

    results = metric_collector.get_results()
    return [calculate_avg_delay(results), calculate_msgs_per_s(results), calculate_avg_step_time(results)]


def calculate_avg_delay(results):
    times: List[int] = []
    active_vehicles: Set[str] = set()
    start_times: Dict[str, float] = {}
    for time, vehicles in zip(results[Metric.TIME], results[Metric.ALL_VEHICLE_IDS]):
        for v in vehicles - active_vehicles:
            start_times[v] = time
        for v in active_vehicles - vehicles:
            times.append(time - start_times[v])
        active_vehicles = vehicles

    unhindered_time = 600 / 13.89
    return np.mean([time - unhindered_time for time in times])


def calculate_msgs_per_s(results):
    last_second = 0
    messages_this_second = 0
    messages_per_second = []
    for time, num_messages in zip(results[Metric.TIME], results[Metric.MESSAGES_EXCHANGED]):
        if time - last_second < 1:
            messages_this_second += num_messages
        else:
            messages_per_second.append(messages_this_second + num_messages)
            messages_this_second = 0
            last_second += 1
    return np.mean(messages_per_second)


def calculate_avg_step_time(results):
    return np.mean(results[Metric.WALL_TIME])


if __name__ == '__main__':
    main()
