from __future__ import annotations
from abc import ABC, abstractmethod

'''
Communication
-------------
Messages can be exchanged between CommunicativeAgents
'''


class Message:
    """
    A class to represent a Message

    Attributes
    ----------
    sender : CommunicativeAgent
        The sender of the message
    contents : dict
        A dictionary containing the message contents.
    """

    def __init__(self, sender: CommunicativeAgent, contents: dict):
        self.sender = sender
        self.contents = contents


class CommunicativeAgent(ABC):
    @abstractmethod
    def send(self, message: Message):
        pass


'''
Intersection Manager
-------------
Communicates with Vehicles and coordinates their movements through the intersection
'''


class IntersectionManager(CommunicativeAgent, ABC):
    def __init__(self, env_interface: IMEnvironmentInterface):
        self.env_interface = env_interface

    @abstractmethod
    def step(self):
        pass


class IMEnvironmentInterface(ABC):
    pass


'''
Vehicles
-------------
Either communicate with each-other or the intersection manager in order to figure out how to proceed through the
intersection
'''


class Vehicle(CommunicativeAgent, ABC):
    def __init__(self, env_interface: VehicleEnvironmentInterface):
        self.env_interface = env_interface
        self.state = VehicleState.DEFAULT

    @abstractmethod
    def step(self):
        pass


class VehicleEnvironmentInterface(ABC):
    @abstractmethod
    def approaching(self, communication_range: int) -> bool:
        pass

    @abstractmethod
    def departing(self) -> bool:
        pass

    @abstractmethod
    def in_intersection(self) -> bool:
        pass


class VehicleState:
    APPROACHING = 0
    IN_INTERSECTION = 1
    DEFAULT = 2
