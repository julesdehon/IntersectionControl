from __future__ import annotations

from abc import ABC, abstractmethod

'''
Communication
-------------
Messages can be exchanged between CommunicativeAgents
'''


class MessagingUnit:
    pass


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
        raise NotImplementedError

    @abstractmethod
    def get_id(self) -> str:
        raise NotImplementedError
