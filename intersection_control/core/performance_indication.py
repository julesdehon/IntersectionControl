from typing import List, Any, Dict, Callable, Tuple, Optional, Set, Type
from timeit import default_timer as timer
import numpy as np
from intersection_control.core import Environment, MessagingUnit


class Metric:
    TIME = 0
    ALL_SPEEDS = 1
    AVG_SPEED = 2
    ALL_ACCELERATIONS = 3
    AVG_ACCELERATION = 4
    ALL_POSITIONS = 5
    NUM_VEHICLES = 6
    NUM_VEHICLES_BY_TRAJECTORY = 7
    ALL_VEHICLE_IDS = 8
    WALL_TIME = 9
    MESSAGES_EXCHANGED = 10


class MetricCollector:
    def __init__(self, environment: Environment, *metrics, messaging_unit_class: Optional[Type[MessagingUnit]] = None):
        self.environment = environment
        self.results: Dict[int, List[Any]] = {metric: [] for metric in metrics}
        self.poll_function: Dict[int, Callable[[], Any]] = {
            Metric.TIME: self._poll_time,
            Metric.ALL_SPEEDS: self._poll_all_speeds,
            Metric.AVG_SPEED: self._poll_avg_speed,
            Metric.ALL_ACCELERATIONS: self._poll_all_accelerations,
            Metric.AVG_ACCELERATION: self._poll_avg_acceleration,
            Metric.ALL_POSITIONS: self._poll_all_positions,
            Metric.NUM_VEHICLES: self._poll_num_vehicles,
            Metric.NUM_VEHICLES_BY_TRAJECTORY: self._poll_num_vehicles_by_trajectory,
            Metric.ALL_VEHICLE_IDS: self._poll_all_vehicle_ids,
            Metric.WALL_TIME: self._poll_wall_time,
            Metric.MESSAGES_EXCHANGED: self._poll_messages_exchanged,
        }
        if Metric.WALL_TIME in metrics:
            self.last_wall_time: Optional[float] = None

        if Metric.MESSAGES_EXCHANGED in metrics:
            assert messaging_unit_class is not None
            self.last_num_send_calls = 0
            self.num_send_calls = 0
            # A little bit of python magic to be able to count the number of calls to send()
            messaging_unit_class.send = self._count_send_calls(messaging_unit_class.send)

    def poll(self):
        for metric, result in self.results.items():
            result.append(self.poll_function[metric]())

    def get_results(self) -> Dict[int, List[Any]]:
        results = self.results
        self.results = {metric: [] for metric in self.results.keys()}
        return results

    def _poll_time(self) -> float:
        return self.environment.get_current_time()

    def _poll_all_speeds(self) -> List[float]:
        return [self.environment.vehicles.get_speed(v) for v in self.environment.vehicles.get_ids()]

    def _poll_avg_speed(self) -> Optional[float]:
        speeds = self._poll_all_speeds()
        return np.mean(speeds) if len(speeds) > 0 else None

    def _poll_all_accelerations(self) -> List[float]:
        return [self.environment.vehicles.get_acceleration(v) for v in self.environment.vehicles.get_ids()]

    def _poll_avg_acceleration(self) -> Optional[float]:
        accelerations = self._poll_all_accelerations()
        return np.mean(accelerations) if len(accelerations) > 0 else None

    def _poll_all_positions(self) -> List[Tuple[float, float]]:
        return [self.environment.vehicles.get_position(v) for v in self.environment.vehicles.get_ids()]

    def _poll_num_vehicles(self) -> int:
        return len(self.environment.vehicles.get_ids())

    def _poll_num_vehicles_by_trajectory(self) -> Dict[str, int]:
        all_trajectories = [item for sublist in [self.environment.intersections.get_trajectories(i) for i in
                                                 self.environment.intersections.get_ids()] for item in sublist]
        result = {trajectory: 0 for trajectory in all_trajectories}
        for v in self.environment.vehicles.get_ids():
            result[self.environment.vehicles.get_trajectory(v)] += 1
        return result

    def _poll_all_vehicle_ids(self) -> Set[str]:
        return set(self.environment.vehicles.get_ids())

    def _poll_wall_time(self) -> float:
        time = timer()
        result = (time - self.last_wall_time) if self.last_wall_time is not None else 0
        self.last_wall_time = time
        return result

    def _poll_messages_exchanged(self) -> int:
        result = self.num_send_calls - self.last_num_send_calls
        self.last_num_send_calls = self.num_send_calls
        return result

    def _count_send_calls(self, send):
        def wrapper(instance, address, message):
            self.num_send_calls += 1
            send(instance, address, message)

        return wrapper
