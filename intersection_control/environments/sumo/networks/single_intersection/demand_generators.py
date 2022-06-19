from typing import List
from intersection_control.environments.sumo import DemandGenerator, NewVehicleParams


class ConflictingDemandGenerator(DemandGenerator):
    """Will spawn vehicles on different routes at times that should cause
    them to conflict when they arrive at the intersection

    Can be used to make sure the intersection control algorithm resolves
    this conflict.
    """

    def __init__(self):
        self.current_id = 0
        self.step_count = 0

    def step(self) -> List[NewVehicleParams]:
        self.step_count += 1
        if self.step_count % 200 != 0:
            return []

        next_id = self.get_next_id()
        conflicting = [("EW", 9), ("NW", 9.5), ("SW", 10)]

        return [NewVehicleParams(f"{route}{next_id}", route, depart_speed=depart_speed) for route, depart_speed in
                conflicting]

    def get_next_id(self) -> str:
        result = self.current_id
        self.current_id += 1
        return str(result)
