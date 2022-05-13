from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List, Dict, Any


class MessagingUnit(ABC):
    """A class to represent a communication system

    An intersection manager or vehicle can have a MessagingUnit, and use
    it to communicate with other vehicles or intersection managers. This class
    can be subclassed to provide different behaviours of varying complexity.
    For example, an implementation that uses a discrete event simulator
    such as Omnet++ or ns-3 to track node connectivity and provide complex
    routing behaviour.

    As a general rule, all MessagingUnits in an environment/experiment
    should be of the same type - although this API would allow for the
    implementation of different Unit types that behave slightly differently
    but are able to communicate.
    """
    @abstractmethod
    def discover(self) -> List[str]:
        """Returns a list of addresses that can be reached

        :return: A list of addresses that can be reached at this
            point in time
        """
        raise NotImplementedError

    @abstractmethod
    def send(self, address: str, message: Message):
        """Sends message to the MessagingUnit with the given address

        :param str address: The address of the MessagingUnit that the
            message should be sent to
        :param Message message: The message to be sent
        """
        raise NotImplementedError

    @abstractmethod
    def receive(self) -> List[Message]:
        """Returns a list of messages received since the last time
        this method was called

        :return: A list of received messages
        """
        raise NotImplementedError

    @abstractmethod
    def broadcast(self, message: Message):
        """Sends a message to all reachable MessagingUnits

        :param Message message: The message to be broadcast
        """
        raise NotImplementedError


class Message:
    """A class to represent a Message

    :ivar sender: The address of the sender of the message
    :ivar contents: A dictionary containing the message contents.
    """
    def __init__(self, sender: str, contents: Dict[str, Any]):
        self.sender = sender
        self.contents = contents
