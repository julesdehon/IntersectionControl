from typing import Set, Tuple, List

from intersection_control.core import IntersectionManager, Environment


class TLIntersectionManager(IntersectionManager):
    def __init__(self, intersection_id: str, environment: Environment,
                 phases: List[Tuple[Tuple[Set[str], Set[str], Set[str]], float]]):
        super().__init__(intersection_id, environment)
        self.phases = phases
        self.phase_index = 0
        self.set_traffic_light_phase(self.phases[self.phase_index][0])
        self.phase_start = self.environment.get_current_time()

    def step(self):
        (_, duration) = self.phases[self.phase_index]
        if self.environment.get_current_time() - self.phase_start >= duration:
            self._switch_to_next_phase()

    def _switch_to_next_phase(self):
        self.phase_index += 1
        self.phase_index = self.phase_index % len(self.phases)
        self.set_traffic_light_phase(self.phases[self.phase_index][0])
        self.phase_start = self.environment.get_current_time()
