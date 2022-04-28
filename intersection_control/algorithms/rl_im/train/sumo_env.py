from typing import Tuple, Dict

from ray.rllib.env.multi_agent_env import MultiAgentEnv
from ray.rllib.utils.typing import MultiAgentDict
import gym.spaces as spaces

from intersection_control.algorithms.rl_im.RLVehicle import RLVehicle
from intersection_control.environments import SumoEnvironment
from intersection_control.environments.sumo.networks.single_intersection.demand_generators import RandomDemandGenerator


class MultiAgentSumoEnv(MultiAgentEnv):
    def __init__(self, env_config):
        super().__init__()

        demand_generator = RandomDemandGenerator({
            "NE": 2, "NS": 2, "NW": 2, "EN": 2, "ES": 2, "EW": 2, "SN": 2, "SE": 2, "SW": 2, "WN": 2, "WE": 2, "WS": 2
        }, 0.1)
        self.env = SumoEnvironment("../../../environments/sumo/networks/single_intersection/intersection.sumocfg",
                                   demand_generator, 0.1, False)
        self.vehicles: Dict[str, RLVehicle] = {vehicle_id: RLVehicle(vehicle_id, self.env) for vehicle_id in
                                               self.env.vehicles.get_ids()}

        self.simulation_time = env_config["simulation_time"]

    @property
    def observation_space(self) -> spaces.Space:
        """Returns the observation space (assuming heterogeneous agents)

        Some slightly more complicated logic would be required to enable
        different agents to have different observation spaces.

        TODO: Choose a sensible observation space

        For now, this is a continuous space between 0 and 100, representing
        the vehicle's speed
        """
        return spaces.Box(0, 100, shape=(1,))

    @property
    def action_space(self) -> spaces.Space:
        """Returns the action space (assuming heterogeneous agents)

        Some slightly more complicated logic would be required to enable
        different agents to have different action spaces.

        TODO: Choose a sensible action space

        For now, there are 3 possible actions: noop, decelerate, accelerate
        """
        return spaces.Discrete(3)

    def reset(self) -> MultiAgentDict:
        self.env.clear()
        return {}

    def step(self, action_dict: MultiAgentDict) \
            -> Tuple[MultiAgentDict, MultiAgentDict, MultiAgentDict, MultiAgentDict]:
        obs = {}
        rewards = {}
        dones = {"__all__": False}  # We are always able to produce more vehicles
        infos = {}

        done_vehicles = self.env.get_removed_vehicles()
        new_vehicles = self.env.get_added_vehicles()

        for vehicle_id, action in action_dict.items():
            if vehicle_id not in done_vehicles:
                self.vehicles[vehicle_id].apply_action(action)
            dones[vehicle_id] = vehicle_id in done_vehicles

        for done_vehicle in done_vehicles:
            del self.vehicles[done_vehicle]

        for new_vehicle in new_vehicles:
            self.vehicles[new_vehicle] = RLVehicle(new_vehicle, self.env)

        for vehicle_id, vehicle in self.vehicles.items():
            obs[vehicle_id] = vehicle.get_observation()
            rewards[vehicle_id] = vehicle.get_reward()
            dones[vehicle_id] = False

        dones["__all__"] = self.env.get_current_time() >= self.simulation_time
        self.env.step()

        return obs, rewards, dones, infos


