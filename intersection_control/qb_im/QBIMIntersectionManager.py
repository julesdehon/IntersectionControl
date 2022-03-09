from intersection_control.interfaces import Message, IntersectionManager, IMEnvironmentInterface


class QBIMIntersectionManager(IntersectionManager):
    def __init__(self, env_interface: IMEnvironmentInterface):
        super().__init__(env_interface)
        self.message_queue = []
        self._interested_vehicle = None

    def step(self):
        pass

    def send(self, message: Message):
        self.message_queue.append(message)
        if self._interested_vehicle is None:
            self._interested_vehicle = message.sender
        if message.sender == self._interested_vehicle:
            print(f"Message received from: {message.sender}\nContents: {message.contents}")

