import traci
import sumolib
import math
from IMSimulationInterface import IMSimulationInterface
from intersection_control.interfaces import VehicleEnvironmentInterface


class VehicleSimulationInterface(VehicleEnvironmentInterface):
    def __init__(self, vehicle_id: str, im_env_interface: IMSimulationInterface):
        self.id = vehicle_id
        self.im_env_interface = im_env_interface

        self.net = sumolib.net.readNet("network/intersection.net.xml")
        self.intersection_incoming_lanes = [edge.getID() for edge in self.net.getNode("intersection").getIncoming()]
        self.intersection_outgoing_lanes = [edge.getID() for edge in self.net.getNode("intersection").getOutgoing()]

    def approaching(self, communication_range) -> bool:
        im_x, im_y = self.im_env_interface.get_position()
        x, y = self.get_position()
        distance = math.sqrt((im_x - x) ** 2 + (im_y - y) ** 2)
        return traci.vehicle.getRoadID(self.id) in self.intersection_incoming_lanes and \
            distance <= communication_range

    def departing(self):
        return traci.vehicle.getRoadID(self.id) in self.intersection_outgoing_lanes

    def in_intersection(self):
        return traci.vehicle.getLaneID(self.id).startswith(":intersection")  # Internal lanes start with a colon (:)

    def get_position(self):
        return traci.vehicle.getPosition(self.id)
