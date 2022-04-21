from typing import Optional
from intersection_control.core import Vehicle, VehicleEnvironmentInterface
from intersection_control.core import IntersectionManager
from intersection_control.core import Message
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
        self.reservation: Optional[Reservation] = None
        self.timeout = self.env_interface.get_current_time()
        self.target_speed = None

    def step(self):
        # Handle all messages
        for message in self.message_queue:
            self.handle_message(message)
        self.message_queue = []

        # If we are within the communication range of the IM, update the vehicle state to APPROACHING
        if self.state == VehicleState.DEFAULT and self.env_interface.approaching(self.communication_range):
            self.state = VehicleState.APPROACHING

        # If we are approaching, and do not yet have a reservation, send a reservation request to the IM
        if self.state == VehicleState.APPROACHING and self.reservation is None \
                and self.env_interface.get_current_time() >= self.timeout:
            logger.debug(f"[{self.get_id()}] Sending reservation request for time {self.approximate_arrival_time()}"
                         f" and velocity {self.approximate_arrival_velocity()}")
            self.intersection_manager.send(Message(self, {
                "type": VehicleMessageType.REQUEST,
                "vehicle_id": self.get_id(),
                "arrival_time": self.approximate_arrival_time(),
                "arrival_lane": self.env_interface.get_trajectory(),
                "arrival_velocity": self.approximate_arrival_velocity(),
                "vehicle_length": self.env_interface.get_length(),
                "vehicle_width": self.env_interface.get_width()
            }))
        elif self.state == VehicleState.APPROACHING and self.reservation is not None \
            and not self.env_interface.in_intersection() \
            and self.env_interface.get_current_time() >= self.timeout \
            and (self.approximate_arrival_time() < self.reservation.early_error
                 or self.approximate_arrival_time() > self.reservation.late_error):
            logger.debug(f"[{self.get_id()}] Changing reservation request. Old reservation time: "
                         f"{self.reservation.arrival_time}, new approximate arrival: {self.approximate_arrival_time()}")
            self.intersection_manager.send(Message(self, {
                "type": VehicleMessageType.CHANGE_REQUEST,
                "vehicle_id": self.get_id(),
                "arrival_time": self.approximate_arrival_time(),
                "arrival_lane": self.env_interface.get_trajectory(),
                "arrival_velocity": self.approximate_arrival_velocity(),
                "vehicle_length": self.env_interface.get_length(),
                "vehicle_width": self.env_interface.get_width(),
                "reservation_id": self.reservation.reservation_id
            }))
            self.reservation = None

        # On arriving at the intersection, produce a log message and update vehicle state to IN_INTERSECTION
        if self.state == VehicleState.APPROACHING and self.env_interface.in_intersection():
            logger.debug(f"[{self.get_id()}] Arrived at intersection. Reservation time: {self.reservation.arrival_time}"
                         f" Actual time: {self.env_interface.get_current_time()}. Reservation velocity: "
                         f"{self.reservation.arrival_velocity} Actual velocity: {self.env_interface.get_velocity()}.")
            self.state = VehicleState.IN_INTERSECTION

        # On departing the intersection, send a DONE message to the IM and update vehicle state to DEFAULT
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
            self.reservation = Reservation(message)
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
    def __init__(self, confirm_message: Message):
        self.reservation_id = confirm_message.contents["reservation_id"]
        self.arrival_time = confirm_message.contents["arrival_time"]
        self.arrival_velocity = confirm_message.contents["arrival_velocity"]
        self.early_error = confirm_message.contents["early_error"]
        self.late_error = confirm_message.contents["late_error"]
