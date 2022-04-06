from intersection_control.interfaces import IntersectionManager, Message, Vehicle, VehicleEnvironmentInterface
from intersection_control.qb_im.constants import VehicleMessageType, IMMessageType, VehicleState
import logging

logger = logging.getLogger(__name__)


class QBIMVehicle(Vehicle):
    def __init__(self, intersection_manager: IntersectionManager, env_interface: VehicleEnvironmentInterface,
                 communication_range: int):
        super().__init__(env_interface)
        self.intersection_manager = intersection_manager
        self.communication_range = communication_range
        self.message_queue = []
        self.state = VehicleState.DEFAULT
        self.reservation: Reservation = None
        self.timeout = self.env_interface.get_current_time()
        self.target_speed = None

    def step(self):
        for message in self.message_queue:
            self.handle_message(message)
        self.message_queue = []

        if self.state == VehicleState.DEFAULT and self.env_interface.approaching(self.communication_range):
            self.state = VehicleState.APPROACHING
        if self.state == VehicleState.APPROACHING and self.reservation is None \
                and self.env_interface.get_current_time() >= self.timeout:
            logger.debug(f"[{self.get_id()}] Sending reservation request")
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
            logger.debug(f"[{self.get_id()}] Arrived at intersection. Reservation time: {self.reservation.arrival_time}"
                         f" Actual time: {self.env_interface.get_current_time()}. Reservation velocity: "
                         f"{self.reservation.arrival_velocity} Actual velocity: {self.env_interface.get_velocity()}.")
            self.state = VehicleState.IN_INTERSECTION
        if self.state == VehicleState.IN_INTERSECTION and self.env_interface.departing():
            logger.debug(f"[{self.get_id()}] Leaving the intersection")
            self.intersection_manager.send(Message(self, {
                "type": VehicleMessageType.DONE
            }))
            self.reservation = None
            self.target_speed = None
            self.env_interface.set_desired_speed(to=-1)
            self.state = VehicleState.DEFAULT

    def get_id(self) -> str:
        return self.env_interface.get_id()

    def send(self, message: Message):
        self.message_queue.append(message)

    def handle_message(self, message: Message):
        if message.contents["type"] == IMMessageType.CONFIRM:
            self.reservation = Reservation(
                message.contents["reservation_id"],
                message.contents["arrival_time"],
                message.contents["arrival_velocity"]
            )
        elif message.contents["type"] == IMMessageType.REJECT:
            self.timeout = message.contents["timeout"]
            self.target_speed = self.env_interface.get_velocity() * 0.8
            self.env_interface.set_desired_speed(to=self.target_speed)
        else:
            logger.warning(f"[{self.get_id()}] Received unknown message type from {message.sender.get_id()}. Ignoring.")

    def approximate_arrival_time(self):
        driving_distance = self.env_interface.get_driving_distance()
        target_speed = self.target_speed if self.target_speed is not None else self.env_interface.get_velocity()
        return self.env_interface.get_current_time() + driving_distance / target_speed

    def approximate_arrival_velocity(self):
        turn_speed_limit = self.env_interface.get_speed_through_trajectory()
        target_speed = self.target_speed if self.target_speed is not None else turn_speed_limit
        return min(self.env_interface.get_velocity(), turn_speed_limit, target_speed)


class Reservation:
    def __init__(self, reservation_id: str, arrival_time: float, arrival_velocity: float):
        self.reservation_id = reservation_id
        self.arrival_time = arrival_time
        self.arrival_velocity = arrival_velocity
