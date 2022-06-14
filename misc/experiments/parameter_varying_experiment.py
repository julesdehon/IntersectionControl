#!/usr/bin/env python
from typing import List, Set, Dict
import numpy as np
import time
from tqdm import tqdm

from intersection_control.algorithms.qb_im import QBIMVehicle, QBIMIntersectionManager
from intersection_control.communication import DistanceBasedUnit
from intersection_control.environments import SumoEnvironment
from intersection_control.core.performance_indication import MetricCollector, Metric
from intersection_control.environments.sumo import RandomDemandGenerator

TIME_STEP = 0.05
VPMs = [0.2, 0.6, 1.]
GRANULARITIES = range(1, 50, 5)
RUNS_PER_GRANULARITY = 3
STEPS_PER_RUN = int((20 * 60) / TIME_STEP)  # 20 minutes
METRICS_TO_COLLECT = [Metric.TIME, Metric.ALL_VEHICLE_IDS]


def main():
    f = open(f"out/{time.time()}-parameter_varying_experiment.csv", "w")
    f.write("vpm,granularity,delay\n")
    for granularity in GRANULARITIES:
        print(f"Running experiments for granularity {granularity}")
        for vpm in VPMs:
            print(f"Running experiments for {vpm} vehicles per minute")
            total = 0
            successful_experiments = 0
            for i in range(RUNS_PER_GRANULARITY):
                print(f"Run {i + 1}/{RUNS_PER_GRANULARITY}")
                results = run_experiment(vpm, granularity)
                if results:
                    total += results
                    successful_experiments += 1
            avg = total / successful_experiments
            f.write(f"{vpm},{granularity},{avg}\n")
            f.flush()
    f.close()


def run_experiment(vpm: float, granularity: int):
    experiment_start = time.time()
    demand_generator = RandomDemandGenerator({
        route: vpm for route in ["NE", "NS", "NW", "EN", "ES", "EW", "SN", "SE", "SW", "WN", "WE", "WS"]
    }, TIME_STEP)
    env = SumoEnvironment(
        "../../intersection_control/environments/sumo/networks/single_intersection/intersection.sumocfg",
        demand_generator=demand_generator, time_step=TIME_STEP, gui=False)

    def v_position_function(vid):
        return lambda: env.vehicles.get_position(vid)

    def im_position_function(imid):
        return lambda: env.intersections.get_position(imid)

    vehicles = {QBIMVehicle(vid, env, DistanceBasedUnit(vid, 75, v_position_function(vid))) for
                vid in env.vehicles.get_ids()}
    intersection_managers = {QBIMIntersectionManager(imid, env, granularity, TIME_STEP,
                                                     DistanceBasedUnit(imid, 75, im_position_function(imid)))
                             for imid in env.intersections.get_ids()}

    metric_collector = MetricCollector(env, *METRICS_TO_COLLECT, messaging_unit_class=DistanceBasedUnit)

    for _ in tqdm(range(STEPS_PER_RUN)):
        env.step()
        removed_vehicles = {v for v in vehicles if v.get_id() in env.get_removed_vehicles()}
        for v in removed_vehicles:
            v.destroy()
        new_vehicles = {QBIMVehicle(vid, env, DistanceBasedUnit(vid, 75, v_position_function(vid)))
                        for vid in env.get_added_vehicles()}
        vehicles = (vehicles - removed_vehicles).union(new_vehicles)
        for vehicle in vehicles:
            vehicle.step()
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
    return calculate_avg_delay(results)


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


if __name__ == '__main__':
    main()
