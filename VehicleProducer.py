import traci
import random

from IMSimulationInterface import IMSimulationInterface
from VehicleSimulationInterface import VehicleSimulationInterface
from intersection_control.interfaces import Vehicle, IntersectionManager
from intersection_control.qb_im.QBIMVehicle import QBIMVehicle


class RandomVehicleProducer:
    def __init__(self, vehicle_list: [Vehicle], rates: dict, algorithm: str, im: IntersectionManager,
                 im_env_interface: IMSimulationInterface):
        self.vehicle_list = vehicle_list
        # convert rates from vehicles per minute to a probability of a spawn event occurring at each time step
        self.spawn_probabilities = {route: (rate * traci.simulation.getDeltaT() / 60, probs)
                                    for route, (rate, probs) in rates.items()}
        self.routes = traci.route.getIDList()
        self.algorithm = algorithm
        self.im = im
        self.im_env_interface = im_env_interface
        self.current_id = 0

    def step(self):
        arrived_vehicles = set(traci.simulation.getArrivedIDList())
        self.vehicle_list[:] = [vehicle for vehicle in self.vehicle_list
                                if vehicle.env_interface.get_id() not in arrived_vehicles]

        for road in self.spawn_probabilities.keys():
            spawn_prob, route_prob = self.spawn_probabilities[road]
            if random.random() <= spawn_prob:
                next_id = self.get_next_id()
                route = random.choices(list(route_prob.keys()), weights=route_prob.values(), k=1)[0]
                traci.vehicle.add(next_id, route, departSpeed="10")
                veh_sim_interface = VehicleSimulationInterface(next_id, self.im_env_interface)
                self.vehicle_list.append(self.vehicle_for_algo(veh_sim_interface))

    def get_next_id(self) -> str:
        result = self.current_id
        self.current_id += 1
        return str(result)

    def vehicle_for_algo(self, veh_sim_interface: VehicleSimulationInterface) -> Vehicle:
        if self.algorithm == "qb-im":
            return QBIMVehicle(self.im, veh_sim_interface, communication_range=75)
        elif self.algorithm == "ab-im":
            raise NotImplementedError
        elif self.algorithm == "decentralised":
            raise NotImplementedError
        else:
            raise ValueError


class ConflictingVehicleProducer:
    def __init__(self, vehicle_list: [Vehicle], algorithm: str, im: IntersectionManager,
                 im_env_interface: IMSimulationInterface):
        self.vehicle_list = vehicle_list
        self.routes = traci.route.getIDList()
        self.algorithm = algorithm
        self.im = im
        self.im_env_interface = im_env_interface
        self.current_id = 0
        self.step_count = 0

    def step(self):
        self.step_count += 1
        arrived_vehicles = set(traci.simulation.getArrivedIDList())
        self.vehicle_list[:] = [vehicle for vehicle in self.vehicle_list
                                if vehicle.env_interface.get_id() not in arrived_vehicles]
        if self.step_count % 200 != 0:
            return

        next_id = self.get_next_id()
        traci.vehicle.add(f"EW{next_id}", "EW", departSpeed="5")
        traci.vehicle.setSpeedMode(f"EW{next_id}", 0b110111)
        veh_sim_interface = VehicleSimulationInterface(f"EW{next_id}", self.im_env_interface)
        self.vehicle_list.append(self.vehicle_for_algo(veh_sim_interface))

        traci.vehicle.add(f"NW{next_id}", "NW", departSpeed="10")
        traci.vehicle.setSpeedMode(f"NW{next_id}", 0b110111)
        veh_sim_interface = VehicleSimulationInterface(f"NW{next_id}", self.im_env_interface)
        self.vehicle_list.append(self.vehicle_for_algo(veh_sim_interface))

        # traci.vehicle.add(f"WN{next_id}", "WN", departSpeed="10")
        # veh_sim_interface = VehicleSimulationInterface(f"WN{next_id}", self.im_env_interface)
        # self.vehicle_list.append(self.vehicle_for_algo(veh_sim_interface))

        traci.vehicle.add(f"SW{next_id}", "SW", departSpeed="10")
        traci.vehicle.setSpeedMode(f"SW{next_id}", 0b110111)
        veh_sim_interface = VehicleSimulationInterface(f"SW{next_id}", self.im_env_interface)
        self.vehicle_list.append(self.vehicle_for_algo(veh_sim_interface))

    def get_next_id(self) -> str:
        result = self.current_id
        self.current_id += 1
        return str(result)

    def vehicle_for_algo(self, veh_sim_interface: VehicleSimulationInterface) -> Vehicle:
        if self.algorithm == "qb-im":
            return QBIMVehicle(self.im, veh_sim_interface, communication_range=75)
        elif self.algorithm == "ab-im":
            raise NotImplementedError
        elif self.algorithm == "decentralised":
            raise NotImplementedError
        else:
            raise ValueError
