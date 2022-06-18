from typing import Optional

from intersection_control.core import Vehicle, Environment, MessagingUnit
from intersection_control.core import Message
from intersection_control.algorithms.qb_im.constants import VehicleMessageType, IMMessageType, VehicleState
import logging

from intersection_control.environments.sumo import ControlType

logger = logging.getLogger(__name__)


class QBIMVehicle(Vehicle):
    def __init__(self, vehicle_id: str, environment: Environment, messaging_unit: MessagingUnit):
        super().__init__(vehicle_id, environment)
        self.messaging_unit = messaging_unit
        self.state = VehicleState.DEFAULT
        self.reservation: Optional[Reservation] = None
        self.timeout = self.environment.get_current_time()
        self.approaching_im = None
        self.target_speed = None
        self.was_just_waiting = False

    def destroy(self):
        self.messaging_unit.destroy()

    def step(self):
        # Handle all messages
        for message in self.messaging_unit.receive():
            self.handle_message(message)

        # If we are within the communication range of the IM, update the vehicle state to APPROACHING
        if self.state == VehicleState.DEFAULT:
            self.approaching_im = self.approaching()
            if self.approaching_im is not None and self.approaching_im in self.messaging_unit.discover():
                self.transition_to_approaching_without_reservation()
        elif self.state == VehicleState.APPROACHING_WITHOUT_RESERVATION:
            if self.get_driving_distance() < self.distance_to_stop() + 1:
                self.transition_to_waiting_at_intersection()
        elif self.state == VehicleState.APPROACHING_WITH_RESERVATION:
            if self.in_intersection():
                self.transition_to_in_intersection()
        elif self.state == VehicleState.IN_INTERSECTION:
            if self.departing():
                self.transition_to_default()

        self.act()

    def transition_to_approaching_without_reservation(self):
        self.state = VehicleState.APPROACHING_WITHOUT_RESERVATION

    def transition_to_approaching_with_reservation(self):
        if self.state == VehicleState.WAITING_AT_INTERSECTION:
            self.target_speed = self.reservation.arrival_velocity
            self.set_desired_speed(self.target_speed)
        self.state = VehicleState.APPROACHING_WITH_RESERVATION

    def transition_to_waiting_at_intersection(self):
        self.target_speed = 0
        self.set_desired_speed(0)
        self.state = VehicleState.WAITING_AT_INTERSECTION

    def transition_to_in_intersection(self):
        assert self.state == VehicleState.APPROACHING_WITH_RESERVATION
        logger.debug(f"[{self.get_id()}] Arrived at intersection. Reservation time: {self.reservation.arrival_time}"
                     f" Actual time: {self.environment.get_current_time()}. Reservation velocity: "
                     f"{self.reservation.arrival_velocity} Actual velocity: {self.get_speed()}.")
        self.state = VehicleState.IN_INTERSECTION
        self.environment.vehicles.set_control_mode(self.get_id(), ControlType.MANUAL)

    def transition_to_default(self):
        assert self.state == VehicleState.IN_INTERSECTION
        logger.debug(f"[{self.get_id()}] Leaving the intersection")
        self.messaging_unit.send(self.approaching_im, Message(self.messaging_unit.address, {
            "type": VehicleMessageType.DONE
        }))
        self.reservation = None
        self.target_speed = None
        self.approaching_im = None
        self.was_just_waiting = False
        self.set_desired_speed(to=-1)
        self.state = VehicleState.DEFAULT
        self.environment.vehicles.set_control_mode(self.get_id(), ControlType.WITH_SAFETY_PRECAUTIONS)

    def handle_message(self, message: Message):
        if message.contents["type"] == IMMessageType.CONFIRM:
            assert self.state == VehicleState.APPROACHING_WITHOUT_RESERVATION \
                   or self.state == VehicleState.WAITING_AT_INTERSECTION
            if self.state == VehicleState.WAITING_AT_INTERSECTION:
                self.was_just_waiting = True
            logger.debug(f"{self.get_id()} Received confirmation from IM")
            self.reservation = Reservation(message)
            self.transition_to_approaching_with_reservation()
        elif message.contents["type"] == IMMessageType.REJECT:
            assert self.state == VehicleState.APPROACHING_WITHOUT_RESERVATION \
                   or self.state == VehicleState.WAITING_AT_INTERSECTION
            logger.debug(f"{self.get_id()} Received rejection from IM")
            self.timeout = message.contents["timeout"]
            if self.state != VehicleState.WAITING_AT_INTERSECTION:
                self.target_speed = max(self.get_speed() * 0.8, 2)
                self.set_desired_speed(to=self.target_speed)
        else:
            logger.warning(f"[{self.get_id()}] Received unknown message type from {message.sender}. Ignoring.")

    def approximate_arrival_time(self):
        target_speed = self.target_speed if self.target_speed is not None else self.get_speed()
        if self.state == VehicleState.WAITING_AT_INTERSECTION or target_speed == 0:
            return self.environment.get_current_time() + \
                   ((2 * self.get_driving_distance()) / self.get_max_acceleration()) ** (1 / 2)
        driving_distance = self.get_driving_distance()
        return self.environment.get_current_time() + driving_distance / target_speed

    def approximate_arrival_velocity(self):
        # TODO: It should get this information by communicating through the intersection manager (or maybe we can
        #  assume this is a static part of the map, and so the vehicle would know it through some sort of google maps)
        # TODO: Maybe Vehicle::get_trajectory should return an actual trajectory rather than a string trajectory id
        if self.state == VehicleState.WAITING_AT_INTERSECTION:
            return (2 * self.get_driving_distance() * self.get_max_acceleration()) ** (1 / 2)
        turn_speed_limit = self.environment.intersections.get_trajectories(self.approaching_im)[
            self.get_trajectory()].speed_limit
        target_speed = self.target_speed if self.target_speed is not None else turn_speed_limit
        return min(self.get_speed(), turn_speed_limit, target_speed)

    def act(self):
        # If we are approaching, and do not yet have a reservation, send a reservation request to the IM
        if self.state == VehicleState.APPROACHING_WITHOUT_RESERVATION \
                or self.state == VehicleState.WAITING_AT_INTERSECTION:
            if self.environment.get_current_time() >= self.timeout:
                logger.debug(f"[{self.get_id()}] Sending reservation request for time {self.approximate_arrival_time()}"
                             f" and velocity {self.approximate_arrival_velocity()}")
                self.messaging_unit.send(self.approaching_im, self.request_message())
        elif self.state == VehicleState.APPROACHING_WITH_RESERVATION:
            if self.environment.get_current_time() >= self.timeout and self.reservation_needs_changing() \
                    and not self.was_just_waiting and self.get_driving_distance() >= self.distance_to_stop():
                logger.debug(f"[{self.get_id()}] Changing reservation request. Old reservation time: "
                             f"{self.reservation.arrival_time}, new approximate arrival: "
                             f"{self.approximate_arrival_time()}")
                self.messaging_unit.send(self.approaching_im, self.change_request_message())
                self.reservation = None
                if self.get_driving_distance() < self.distance_to_stop() + 5:
                    self.transition_to_waiting_at_intersection()
                else:
                    self.transition_to_approaching_without_reservation()
        elif self.state == VehicleState.IN_INTERSECTION:
            if self.reservation.accelerate:
                self.set_desired_speed(self.get_speed_limit())

    def reservation_needs_changing(self) -> bool:
        return self.approximate_arrival_time() < self.reservation.early_error \
               or self.approximate_arrival_time() > self.reservation.late_error

    def distance_to_stop(self) -> float:
        return self.get_speed() ** 2 / (2 * self.get_max_deceleration())

    def request_message(self) -> Message:
        return Message(self.messaging_unit.address, {
            "type": VehicleMessageType.REQUEST,
            "vehicle_id": self.get_id(),
            "arrival_time": self.approximate_arrival_time(),
            "arrival_lane": self.get_trajectory(),
            "arrival_velocity": self.approximate_arrival_velocity(),
            "maximum_acceleration": self.get_max_acceleration(),
            "maximum_velocity": self.get_speed_limit(),
            "vehicle_length": self.get_length(),
            "vehicle_width": self.get_width(),
            "distance": self.get_driving_distance(),
        })

    def change_request_message(self) -> Message:
        return Message(self.messaging_unit.address, {
            "type": VehicleMessageType.CHANGE_REQUEST,
            "vehicle_id": self.get_id(),
            "arrival_time": self.approximate_arrival_time(),
            "arrival_lane": self.get_trajectory(),
            "arrival_velocity": self.approximate_arrival_velocity(),
            "maximum_acceleration": self.get_max_acceleration(),
            "maximum_velocity": self.get_speed_limit(),
            "vehicle_length": self.get_length(),
            "vehicle_width": self.get_width(),
            "distance": self.get_driving_distance(),
            "reservation_id": self.reservation.reservation_id,
        })


class Reservation:
    def __init__(self, confirm_message: Message):
        self.reservation_id = confirm_message.contents["reservation_id"]
        self.arrival_time = confirm_message.contents["arrival_time"]
        self.arrival_velocity = confirm_message.contents["arrival_velocity"]
        self.early_error = confirm_message.contents["early_error"]
        self.late_error = confirm_message.contents["late_error"]
        self.accelerate = confirm_message.contents["accelerate"]
