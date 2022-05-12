from typing import Tuple, Dict

from ray.rllib.env.multi_agent_env import MultiAgentEnv
from ray.rllib.utils.typing import MultiAgentDict
import gym.spaces as spaces

from intersection_control.misc.single_lane.simple_rl_vehicle import SimpleRLVehicle
from intersection_control.environments import SumoEnvironment
from intersection_control.environments.sumo import ScenarioGenerator
from intersection_control.environments.sumo.sumo_environment import NewVehicleParams, ControlType


class SingleRoadEnv(MultiAgentEnv):
    def __init__(self, _, gui=False, env_steps_per_step=10):
        super().__init__()

        self.env_steps_per_step = env_steps_per_step

        initial_vehicle_spawns = [
            NewVehicleParams("v_0", "r_0", 5, 0, ControlType.MANUAL),
            NewVehicleParams("v_1", "r_0", 5, 75, ControlType.MANUAL),
            NewVehicleParams("v_2", "r_0", 5, 150, ControlType.MANUAL),
            NewVehicleParams("v_3", "r_0", 5, 225, ControlType.MANUAL)
        ]
        self.demand_generator = ScenarioGenerator(initial_vehicle_spawns)
        self.env = SumoEnvironment("../../environments/sumo/networks/single_road/single_road.sumocfg",
                                   demand_generator=self.demand_generator, time_step=0.1, gui=gui, warnings=False)
        self.env.step()  # Load all the vehicles in

        self.vehicles: Dict[str, SimpleRLVehicle] = {vehicle_id: SimpleRLVehicle(vehicle_id, self.env) for vehicle_id in
                                                     self.env.vehicles.get_ids()}

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
        self.demand_generator.reset()
        self.env.step()  # Load all the vehicles in
        self.vehicles: Dict[str, SimpleRLVehicle] = {vehicle_id: SimpleRLVehicle(vehicle_id, self.env) for vehicle_id in
                                                     self.env.vehicles.get_ids()}

        return {vehicle_id: vehicle.get_observation() for vehicle_id, vehicle in self.vehicles.items()}

    def step(self, action_dict: MultiAgentDict) \
            -> Tuple[MultiAgentDict, MultiAgentDict, MultiAgentDict, MultiAgentDict]:

        for v, action in action_dict.items():
            self.vehicles[v].apply_action(action)

        for _ in range(self.env_steps_per_step):
            self.env.step()

            if len(self.env.get_removed_vehicles()) > 0:
                obs = {vehicle: self.observation_space.sample() for vehicle in self.vehicles}
                rewards = {vehicle: 0 for vehicle in self.vehicles}
                dones = {vehicle: True for vehicle in self.vehicles}
                infos = {vehicle: {} for vehicle in self.vehicles}
                dones["__all__"] = True
                return obs, rewards, dones, infos

        obs = {vehicle_id: vehicle.get_observation() for vehicle_id, vehicle in self.vehicles.items()}
        rewards = {vehicle_id: vehicle.get_reward() for vehicle_id, vehicle in self.vehicles.items()}
        dones = {vehicle_id: False for vehicle_id, vehicle in self.vehicles.items()}
        infos = {vehicle_id: {} for vehicle_id, vehicle in self.vehicles.items()}
        dones["__all__"] = False

        return obs, rewards, dones, infos
