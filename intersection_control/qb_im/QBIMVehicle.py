from intersection_control.interfaces import IntersectionManager, Message, Vehicle, VehicleState, \
    VehicleEnvironmentInterface


class QBIMVehicle(Vehicle):
    def __init__(self, intersection_manager: IntersectionManager, env_interface: VehicleEnvironmentInterface,
                 communication_range: int):
        super().__init__(env_interface)
        self.intersection_manager = intersection_manager
        self.communication_range = communication_range
        self.message_queue = []

    def step(self):
        if self.state == VehicleState.DEFAULT and self.env_interface.approaching(self.communication_range):
            self.intersection_manager.send(Message(self, {"contents": "I am approaching you!"}))
            self.state = VehicleState.APPROACHING
        if self.state == VehicleState.APPROACHING and self.env_interface.in_intersection():
            self.intersection_manager.send(Message(self, {"contents": "I am now inside the intersection!"}))
            self.state = VehicleState.IN_INTERSECTION
        if self.state == VehicleState.IN_INTERSECTION and self.env_interface.departing():
            self.intersection_manager.send(Message(self, {"contents": "I am leaving the intersection!"}))
            self.state = VehicleState.DEFAULT

    def send(self, message: Message):
        self.message_queue.append(message)
