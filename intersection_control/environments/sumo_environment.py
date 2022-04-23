from intersection_control.core import Environment
from intersection_control.core.environment import VehicleHandler, IntersectionHandler


class SumoEnvironment(Environment):
    def __init__(self):
        self._intersections = SumoIntersectionHandler()
        self._vehicles = SumoVehicleHandler()

    @property
    def intersections(self) -> IntersectionHandler:
        return self._intersections

    @property
    def vehicles(self) -> VehicleHandler:
        return self._vehicles

    def get_current_time(self) -> float:
        pass

    def step(self):
        pass
