import math
from typing import List, Dict, Optional, Tuple
import sumolib
import traci
import traci.constants as tc
from intersection_control.core.environment import VehicleHandler
from .utils import ControlType


class SumoVehicleHandler(VehicleHandler):
    def __init__(self, net: sumolib.net.Net):
        self.net = net

        # Dictionary mapping roads to the intersections that they enter
        self.intersection_entered_by_lane = self._get_intersections_entered_by_lanes()

        # Dictionary mapping roads to the intersections that they exit
        self.intersection_exited_by_lane = self._get_intersections_exited_by_lanes()

        # Dictionary mapping lanes to the intersection they are inside of
        self.intersection_containing_lane = self._get_intersections_containing_lanes()

        self.subscription_results = None

    def approaching(self, vehicle_id: str) -> Optional[str]:
        return self.intersection_entered_by_lane.get(self.subscription_results[vehicle_id][tc.VAR_ROAD_ID])

    def departing(self, vehicle_id: str) -> Optional[str]:
        return self.intersection_exited_by_lane.get(self.subscription_results[vehicle_id][tc.VAR_ROAD_ID])

    def in_intersection(self, vehicle_id: str) -> Optional[str]:
        return self.intersection_containing_lane.get(self.subscription_results[vehicle_id][tc.VAR_LANE_ID])

    def get_ids(self) -> List[str]:
        return traci.vehicle.getIDList()

    def get_trajectory(self, vehicle_id: str) -> str:
        return f"{self.subscription_results[vehicle_id][tc.VAR_ROUTE_ID]}-" \
               f"{self.subscription_results[vehicle_id][tc.VAR_LANE_ID]}"

    def get_length(self, vehicle_id: str) -> float:
        return self.subscription_results[vehicle_id][tc.VAR_LENGTH]

    def get_width(self, vehicle_id: str) -> float:
        return self.subscription_results[vehicle_id][tc.VAR_WIDTH]

    def get_driving_distance(self, vehicle_id: str) -> float:
        return self.net.getLane(self.subscription_results[vehicle_id][tc.VAR_LANE_ID]).getLength() - \
               self.subscription_results[vehicle_id][tc.VAR_LANEPOSITION]

    def get_speed(self, vehicle_id: str) -> float:
        return self.subscription_results[vehicle_id][tc.VAR_SPEED]

    def get_position(self, vehicle_id) -> Tuple[float, float]:
        return self.subscription_results[vehicle_id][tc.VAR_POSITION]

    def get_direction(self, vehicle_id) -> float:
        # Transform as required
        return math.pi - (math.radians(self.subscription_results[vehicle_id][tc.VAR_ANGLE]) + math.pi / 2) % (
                2 * math.pi)

    def set_desired_speed(self, vehicle_id: str, to: float):
        traci.vehicle.setSpeed(vehicle_id, to)

    def get_speed_limit(self, vehicle_id) -> float:
        return self.subscription_results[vehicle_id][tc.VAR_ALLOWED_SPEED]

    def get_acceleration(self, vehicle_id: str) -> float:
        return self.subscription_results[vehicle_id][tc.VAR_ACCELERATION]

    def get_max_acceleration(self, vehicle_id: str) -> float:
        return self.subscription_results[vehicle_id][tc.VAR_ACCEL]

    def get_max_deceleration(self, vehicle_id: str) -> float:
        return self.subscription_results[vehicle_id][tc.VAR_DECEL]

    def _get_intersections_entered_by_lanes(self) -> Dict[str, str]:
        intersections = [node for node in self.net.getNodes() if node.getType() == "traffic_light"]
        result = {}
        for intersection in intersections:
            for edge in [edge for edge in intersection.getIncoming() if edge.getFunction() != "internal"]:
                result[edge.getID()] = intersection.getID()
        return result

    def _get_intersections_exited_by_lanes(self) -> Dict[str, str]:
        intersections = [node for node in self.net.getNodes() if node.getType() == "traffic_light"]
        result = {}
        for intersection in intersections:
            for edge in [edge for edge in intersection.getOutgoing() if edge.getFunction() != "internal"]:
                result[edge.getID()] = intersection.getID()
        return result

    def _get_intersections_containing_lanes(self) -> Dict[str, str]:
        intersections = [node for node in self.net.getNodes() if node.getType() == "traffic_light"]
        result = {}
        for intersection in intersections:
            for lane in intersection.getInternal():
                result[lane] = intersection.getID()
        return result

    def set_control_mode(self, vehicle_id, control_type: ControlType):
        if control_type == ControlType.MANUAL:
            traci.vehicle.setSpeedMode(vehicle_id, 0b100110)
        elif control_type == ControlType.WITH_SAFETY_PRECAUTIONS:
            traci.vehicle.setSpeedMode(vehicle_id, 31)
