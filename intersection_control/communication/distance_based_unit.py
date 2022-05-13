from __future__ import annotations
from typing import List, Tuple, Dict, Callable
from math import sqrt

from intersection_control.core import MessagingUnit, Message


class DistanceBasedUnit(MessagingUnit):
    """
    A unit that is able to send messages to other units within a fixed radius,
    provided by the communication_range parameter.

    Requires passing a function, get_position, which returns the position of
    the unit in space.

    This unit should perform well enough when there is a reasonably small number
    of units. For large simulations with thousands of units, a more complex
    implementation may be required.

    .. warning::
        :func:`__del__` must be called explicitly when the unit goes out of scope
        (the IM or vehicle using it goes out of scope) - otherwise it will never
        be removed from the network.
    """
    _network: Dict[str, DistanceBasedUnit] = {}

    def __init__(self, address: str, communication_range: float, get_position: Callable[[], Tuple[float, float]]):
        # assert address not in self._network
        self.address = address
        self.get_position = get_position
        self.communication_range = communication_range
        self._network[self.address] = self
        self._message_queue: List[Message] = []

    def destroy(self):
        """This must be called explicitly when the unit goes out of scope
        (the IM or vehicle using it goes out of scope) - otherwise it will never
        be removed from the network."""
        if self.address in self._network:
            del self._network[self.address]

    def discover(self) -> List[str]:
        return [address for address, unit in list(self._network.items()) if self._within_range(unit)]

    def send(self, address: str, message: Message):
        other_unit = self._network.get(address)
        assert other_unit is not None and self._within_range(other_unit)
        other_unit._pass_message(message)

    def receive(self) -> List[Message]:
        messages = self._message_queue
        self._message_queue = []
        return messages

    def broadcast(self, message: Message):
        for address in self.discover():
            if address == self.address:
                continue
            self.send(address, message)

    def _within_range(self, other_unit: DistanceBasedUnit):
        x, y = self.get_position()
        x1, y1 = other_unit.get_position()
        return sqrt((x - x1) ** 2 + (y - y1) ** 2) < self.communication_range

    def _pass_message(self, message: Message):
        self._message_queue.append(message)
