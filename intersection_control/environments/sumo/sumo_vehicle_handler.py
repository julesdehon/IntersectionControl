import math
from typing import List, Dict, Optional, Tuple
import sumolib
import traci
from intersection_control.core.environment import VehicleHandler


class SumoVehicleHandler(VehicleHandler):
    def __init__(self, net_file: str):
        self.net = sumolib.net.readNet(net_file, withInternal=True)

        # Dictionary mapping roads to the intersections that they enter
        self.intersection_entered_by_lane = self._get_intersections_entered_by_lanes()

        # Dictionary mapping roads to the intersections that they exit
        self.intersection_exited_by_lane = self._get_intersections_exited_by_lanes()

        # Dictionary mapping lanes to the intersection they are inside of
        self.intersection_containing_lane = self._get_intersections_containing_lanes()

    def approaching(self, vehicle_id: str) -> Optional[str]:
        return self.intersection_entered_by_lane.get(traci.vehicle.getRoadID(vehicle_id))

    def departing(self, vehicle_id: str) -> Optional[str]:
        return self.intersection_exited_by_lane.get(traci.vehicle.getRoadID(vehicle_id))

    def in_intersection(self, vehicle_id: str) -> Optional[str]:
        return self.intersection_containing_lane.get(traci.vehicle.getLaneID(vehicle_id))

    def get_ids(self) -> List[str]:
        return traci.vehicle.getIDList()

    def get_trajectory(self, vehicle_id: str) -> str:
        return traci.vehicle.getRouteID(vehicle_id)

    def get_length(self, vehicle_id: str) -> float:
        return traci.vehicle.getLength(vehicle_id)

    def get_width(self, vehicle_id: str) -> float:
        return traci.vehicle.getLength(vehicle_id)

    def get_driving_distance(self, vehicle_id: str) -> float:
        road_end_x, road_end_y = self.net.getEdge(traci.vehicle.getRoadID(vehicle_id)).getShape()[-1]
        return traci.vehicle.getDrivingDistance2D(vehicle_id, road_end_x, road_end_y)

    def get_speed(self, vehicle_id: str) -> float:
        return traci.vehicle.getSpeed(vehicle_id)

    def get_position(self, vehicle_id) -> Tuple[float, float]:
        return traci.vehicle.getPosition(vehicle_id)

    def get_direction(self, vehicle_id) -> float:
        return (math.radians(traci.vehicle.getAngle(vehicle_id)) - math.pi / 2) % (2 * math.pi)  # Transform as required

    def set_desired_speed(self, vehicle_id: str, to: float):
        traci.vehicle.setSpeed(vehicle_id, to)

    def get_speed_limit(self, vehicle_id) -> float:
        return traci.vehicle.getAllowedSpeed(vehicle_id)

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
