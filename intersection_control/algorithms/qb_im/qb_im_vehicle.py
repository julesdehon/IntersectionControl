from typing import Optional
from intersection_control.core import Vehicle, Environment
from intersection_control.core import IntersectionManager
from intersection_control.core import Message
from intersection_control.algorithms.qb_im.constants import VehicleMessageType, IMMessageType, VehicleState
import logging

logger = logging.getLogger(__name__)


class QBIMVehicle(Vehicle):
    def __init__(self, vehicle_id: str, intersection_manager: IntersectionManager, environment: Environment,
                 communication_range: int):
        super().__init__(vehicle_id, environment)
        self.intersection_manager = intersection_manager
        self.communication_range = communication_range
        self.message_queue = []
        self.state = VehicleState.DEFAULT
        self.reservation: Optional[Reservation] = None
        self.timeout = self.environment.get_current_time()
        self.target_speed = None

    def step(self):
        # Handle all messages
        for message in self.message_queue:
            self.handle_message(message)
        self.message_queue = []

        # If we are within the communication range of the IM, update the vehicle state to APPROACHING
        # TODO: Reimplement communication range when communicating with the intersection manager
        if self.state == VehicleState.DEFAULT and self.approaching():
            self.state = VehicleState.APPROACHING

        # If we are approaching, and do not yet have a reservation, send a reservation request to the IM
        if self.state == VehicleState.APPROACHING and self.reservation is None \
                and self.environment.get_current_time() >= self.timeout:
            logger.debug(f"[{self.get_id()}] Sending reservation request for time {self.approximate_arrival_time()}"
                         f" and velocity {self.approximate_arrival_velocity()}")
            self.intersection_manager.send(Message(self, {
                "type": VehicleMessageType.REQUEST,
                "vehicle_id": self.get_id(),
                "arrival_time": self.approximate_arrival_time(),
                "arrival_lane": self.get_trajectory(),
                "arrival_velocity": self.approximate_arrival_velocity(),
                "vehicle_length": self.get_length(),
                "vehicle_width": self.get_width()
            }))
        elif self.state == VehicleState.APPROACHING and self.reservation is not None \
                and not self.in_intersection() \
                and self.environment.get_current_time() >= self.timeout \
                and (self.approximate_arrival_time() < self.reservation.early_error
                     or self.approximate_arrival_time() > self.reservation.late_error):
            logger.debug(f"[{self.get_id()}] Changing reservation request. Old reservation time: "
                         f"{self.reservation.arrival_time}, new approximate arrival: {self.approximate_arrival_time()}")
            self.intersection_manager.send(Message(self, {
                "type": VehicleMessageType.CHANGE_REQUEST,
                "vehicle_id": self.get_id(),
                "arrival_time": self.approximate_arrival_time(),
                "arrival_lane": self.get_trajectory(),
                "arrival_velocity": self.approximate_arrival_velocity(),
                "vehicle_length": self.get_length(),
                "vehicle_width": self.get_width(),
                "reservation_id": self.reservation.reservation_id
            }))
            self.reservation = None

        # On arriving at the intersection, produce a log message and update vehicle state to IN_INTERSECTION
        if self.state == VehicleState.APPROACHING and self.in_intersection():
            logger.debug(f"[{self.get_id()}] Arrived at intersection. Reservation time: {self.reservation.arrival_time}"
                         f" Actual time: {self.environment.get_current_time()}. Reservation velocity: "
                         f"{self.reservation.arrival_velocity} Actual velocity: {self.get_speed()}.")
            self.state = VehicleState.IN_INTERSECTION

        # On departing the intersection, send a DONE message to the IM and update vehicle state to DEFAULT
        if self.state == VehicleState.IN_INTERSECTION and self.departing():
            logger.debug(f"[{self.get_id()}] Leaving the intersection")
            self.intersection_manager.send(Message(self, {
                "type": VehicleMessageType.DONE
            }))
            self.reservation = None
            self.target_speed = None
            self.set_desired_speed(to=-1)
            self.state = VehicleState.DEFAULT

    def get_id(self) -> str:
        return self.vehicle_id

    def send(self, message: Message):
        self.message_queue.append(message)

    def handle_message(self, message: Message):
        if message.contents["type"] == IMMessageType.CONFIRM:
            self.reservation = Reservation(message)
        elif message.contents["type"] == IMMessageType.REJECT:
            self.timeout = message.contents["timeout"]
            self.target_speed = self.get_speed() * 0.8
            self.set_desired_speed(to=self.target_speed)
        else:
            logger.warning(f"[{self.get_id()}] Received unknown message type from {message.sender.get_id()}. Ignoring.")

    def approximate_arrival_time(self):
        driving_distance = self.get_driving_distance()
        target_speed = self.target_speed if self.target_speed is not None else self.get_speed()
        return self.environment.get_current_time() + driving_distance / target_speed

    def approximate_arrival_velocity(self):
        # TODO: It should get this information by communicating through the intersection manager (or maybe we can
        #  assume this is a static part of the map, and so the vehicle would know it through some sort of google maps)
        # TODO: Maybe Vehicle::get_trajectory should return an actual trajectory rather than a string trajectory id
        turn_speed_limit = self.intersection_manager.get_trajectories()[self.get_trajectory()].speed_limit
        target_speed = self.target_speed if self.target_speed is not None else turn_speed_limit
        return min(self.get_speed(), turn_speed_limit, target_speed)


class Reservation:
    def __init__(self, confirm_message: Message):
        self.reservation_id = confirm_message.contents["reservation_id"]
        self.arrival_time = confirm_message.contents["arrival_time"]
        self.arrival_velocity = confirm_message.contents["arrival_velocity"]
        self.early_error = confirm_message.contents["early_error"]
        self.late_error = confirm_message.contents["late_error"]
