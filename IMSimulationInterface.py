from intersection_control.interfaces import IMEnvironmentInterface
import traci


class IMSimulationInterface(IMEnvironmentInterface):
    def __init__(self, intersection_id):
        self.intersection_id = intersection_id

    def get_position(self) -> (float, float):
        return traci.junction.getPosition(self.intersection_id)
