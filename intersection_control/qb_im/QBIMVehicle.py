from intersection_control.interfaces import IntersectionManager, Message, Vehicle, VehicleEnvironmentInterface
from intersection_control.qb_im.constants import VehicleMessageType, IMMessageType, VehicleState


class QBIMVehicle(Vehicle):
    def __init__(self, intersection_manager: IntersectionManager, env_interface: VehicleEnvironmentInterface,
                 communication_range: int):
        super().__init__(env_interface)
        self.intersection_manager = intersection_manager
        self.communication_range = communication_range
        self.message_queue = []
        self.state = VehicleState.DEFAULT

    def step(self):
        for message in self.message_queue:
            self.handle_message(message)
        if self.state == VehicleState.DEFAULT and self.env_interface.approaching(self.communication_range):
            self.intersection_manager.send(Message(self, {
                "type": VehicleMessageType.REQUEST,
                "vehicle_id": self.get_id(),
                "arrival_time": self.approximate_arrival_time(),
                "arrival_lane": self.env_interface.get_trajectory(),
                "arrival_velocity": self.approximate_arrival_velocity(),
                "vehicle_length": self.env_interface.get_length(),
                "vehicle_width": self.env_interface.get_width()
            }))
        if self.state == VehicleState.APPROACHING and self.env_interface.in_intersection():
            self.intersection_manager.send(
                Message(self, {"type": MessageType.ENTERED, "contents": "I am now inside the intersection!"}))
            self.state = VehicleState.IN_INTERSECTION
        if self.state == VehicleState.IN_INTERSECTION and self.env_interface.departing():
            self.intersection_manager.send(
                Message(self, {"type": MessageType.DEPARTING, "contents": "I am leaving the intersection!"}))
            self.state = VehicleState.DEFAULT

    def get_id(self) -> str:
        return self.env_interface.get_id()

    def send(self, message: Message):
        self.message_queue.append(message)

    def handle_message(self, message: Message):
        pass

    def approximate_arrival_time(self):
        pass

    def approximate_arrival_velocity(self):
        pass
