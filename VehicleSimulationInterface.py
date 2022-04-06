import traci
import sumolib
import math
from IMSimulationInterface import IMSimulationInterface
from intersection_control.interfaces import VehicleEnvironmentInterface


class VehicleSimulationInterface(VehicleEnvironmentInterface):
    def __init__(self, vehicle_id: str, im_env_interface: IMSimulationInterface):
        self.id = vehicle_id
        self.im_env_interface = im_env_interface

        self.net = sumolib.net.readNet("network/intersection.net.xml", withInternal=True)
        self.intersection_incoming_lanes = [edge.getID() for edge in self.net.getNode("intersection").getIncoming()]
        self.intersection_outgoing_lanes = [edge.getID() for edge in self.net.getNode("intersection").getOutgoing()]

    def get_trajectory(self):
        return traci.vehicle.getRouteID(self.get_id())

    def get_length(self):
        return traci.vehicle.getLength(self.get_id())

    def get_width(self):
        return traci.vehicle.getWidth(self.get_id())

    def get_driving_distance(self) -> float:
        road_end_x, road_end_y = self.net.getEdge(traci.vehicle.getRoadID(self.get_id())).getShape()[-1]
        return traci.vehicle.getDrivingDistance2D(self.get_id(), road_end_x, road_end_y)

    def get_velocity(self) -> float:
        return traci.vehicle.getSpeed(self.get_id())

    def get_speed_through_trajectory(self) -> float:
        route = [route for route in self.im_env_interface.routes if route.id == self.get_trajectory()][0]
        edges = route.edges.split()
        inner_connection = self.net.getEdge(edges[0]).getConnections(self.net.getEdge(edges[1]))[0]
        return self.net.getLane(inner_connection.getViaLaneID()).getSpeed()

    def approaching(self, communication_range) -> bool:
        im_x, im_y = self.im_env_interface.get_position()
        x, y = self.get_position()
        distance = math.sqrt((im_x - x) ** 2 + (im_y - y) ** 2)
        return traci.vehicle.getRoadID(self.id) in self.intersection_incoming_lanes and \
            distance <= communication_range

    def departing(self):
        return traci.vehicle.getRoadID(self.id) in self.intersection_outgoing_lanes and not self.in_intersection()

    def in_intersection(self):
        return traci.vehicle.getLaneID(self.id).startswith(":intersection")  # Internal lanes start with a colon (:)

    def get_position(self):
        return traci.vehicle.getPosition(self.id)

    def get_id(self) -> str:
        return self.id

    def set_desired_speed(self, to: float):
        traci.vehicle.setSpeed(self.get_id(), to)

    def get_current_time(self) -> float:
        return traci.simulation.getTime()
