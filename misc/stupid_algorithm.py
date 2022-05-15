from intersection_control.core import Vehicle, IntersectionManager, Environment, Message
from intersection_control.communication import DistanceBasedUnit
from intersection_control.environments import SumoEnvironment
from intersection_control.environments.sumo import RandomDemandGenerator
from intersection_control.environments.sumo.sumo_environment import ControlType


class StupidVehicle(Vehicle):
    def __init__(self, vehicle_id: str, environment: Environment):
        super().__init__(vehicle_id, environment)
        self.messaging_unit = DistanceBasedUnit(vehicle_id, 50, self.get_position)

    def step(self):
        # This assumes every intersection manager id will start with "intersection"
        for im in [im for im in self.messaging_unit.discover() if im.startswith("intersection")]:
            self.messaging_unit.send(im, Message(self.vehicle_id, {"speed": self.get_speed()}))
        self.set_desired_speed(self.get_speed() + 1)

    def destroy(self):
        self.messaging_unit.destroy()


class StupidIntersectionManager(IntersectionManager):
    def __init__(self, intersection_id: str, environment: Environment):
        super().__init__(intersection_id, environment)
        self.messaging_unit = DistanceBasedUnit(intersection_id, 50, self.get_position)

    def step(self):
        for message in self.messaging_unit.receive():
            print(f"Received message from {message.sender}: {message.contents}")


def main():
    demand_generator = RandomDemandGenerator({
        "NE": 5, "NS": 5, "NW": 5, "EN": 5, "ES": 5, "EW": 5, "SN": 5, "SE": 5, "SW": 5, "WN": 5, "WE": 5, "WS": 5
    }, 0.05, control_type=ControlType.MANUAL)
    # demand_generator = ConflictingDemandGenerator()
    env = SumoEnvironment("../intersection_control/environments/sumo/networks/single_intersection/intersection.sumocfg",
                          demand_generator=demand_generator, time_step=0.05, gui=True)

    intersection_managers = {StupidIntersectionManager(intersection_id, env) for intersection_id in
                             env.intersections.get_ids()}
    vehicles = {StupidVehicle(vehicle_id, env) for vehicle_id in env.vehicles.get_ids()}

    for _ in range(36000):
        env.step()
        removed_vehicles = {v for v in vehicles if v.get_id() in env.get_removed_vehicles()}
        for v in removed_vehicles:
            v.destroy()
        new_vehicles = {StupidVehicle(vehicle_id, env) for vehicle_id in env.get_added_vehicles()}
        vehicles = (vehicles - removed_vehicles).union(new_vehicles)
        for vehicle in vehicles:
            vehicle.step()
        for intersection_manager in intersection_managers:
            intersection_manager.step()


if __name__ == "__main__":
    main()
