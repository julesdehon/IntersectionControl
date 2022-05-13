import numpy as np
from typing import Tuple, Dict

from ray.rllib.env.multi_agent_env import MultiAgentEnv
from ray.rllib.utils.typing import MultiAgentDict
import gym.spaces as spaces

from intersection_control.algorithms.rl_im.rl_vehicle import RLVehicle
from intersection_control.environments import SumoEnvironment
from intersection_control.environments.sumo import ScenarioGenerator
from intersection_control.environments.sumo.sumo_environment import NewVehicleParams, ControlType


class MultiAgentSumoEnv(MultiAgentEnv):
    def __init__(self, _, gui=False, env_steps_per_step=10):
        super().__init__()

        self.env_steps_per_step = env_steps_per_step
        self.gui = gui

        initial_vehicle_spawns = [
            NewVehicleParams("v_0", "SN", 5, 0, ControlType.MANUAL),
            NewVehicleParams("v_1", "SE", 5, 35, ControlType.MANUAL),
            NewVehicleParams("v_2", "SW", 5, 60, ControlType.MANUAL),

            NewVehicleParams("v_3", "EW", 5, 0, ControlType.MANUAL),
            NewVehicleParams("v_4", "EN", 5, 15, ControlType.MANUAL),
            NewVehicleParams("v_5", "ES", 5, 50, ControlType.MANUAL),

            NewVehicleParams("v_6", "NW", 5, 0, ControlType.MANUAL),
            NewVehicleParams("v_7", "NS", 5, 25, ControlType.MANUAL),
            NewVehicleParams("v_8", "NE", 5, 63, ControlType.MANUAL),

            NewVehicleParams("v_9", "WS", 5, 0, ControlType.MANUAL),
            NewVehicleParams("v_10", "WN", 5, 22, ControlType.MANUAL),
            NewVehicleParams("v_11", "WE", 5, 36, ControlType.MANUAL),
        ]
        self.demand_generator = ScenarioGenerator(initial_vehicle_spawns)
        self.env = SumoEnvironment("../../../environments/sumo/networks/single_intersection/intersection.sumocfg",
                                   demand_generator=self.demand_generator, time_step=0.1, gui=self.gui, warnings=False)
        self.env.step()  # Load all the vehicles in

        self.vehicles: Dict[str, RLVehicle] = {vehicle_id: RLVehicle(vehicle_id, self.env, "intersection") for
                                               vehicle_id in self.env.vehicles.get_ids()}

        for vehicle in self.vehicles.values():
            vehicle.step()  # So they all broadcast their details, ready for the observations

    @property
    def observation_space(self) -> spaces.Space:
        """Returns the observation space (assuming heterogeneous agents)

        The observation space contains information about the ego vehicle, and its 5
        nearest neighbours:
            - x_distance
            - y_distance
            - speed
            - sin(direction)
            - cos(direction)
            - trajectory
            - present (1 if this is an actual vehicle, 0 if it is padding)
        """
        return spaces.Box(-np.inf, np.inf, shape=(6, 7))

    @property
    def action_space(self) -> spaces.Space:
        """Returns the action space (assuming heterogeneous agents)

        Some slightly more complicated logic would be required to enable
        different agents to have different action spaces.

        There are 3 possible actions: noop, decelerate, accelerate
        """
        return spaces.Discrete(3)

    def reset(self) -> MultiAgentDict:
        try:
            self.env.clear()
            self.demand_generator.reset()
            for vehicle in self.vehicles.values():
                vehicle.destroy()
            self.env.step()  # Load all the vehicles in
            self.vehicles = {vehicle_id: RLVehicle(vehicle_id, self.env, "intersection") for
                             vehicle_id in self.env.vehicles.get_ids()}
            for vehicle in self.vehicles.values():
                vehicle.step()  # So they all broadcast their details, ready for the observations
        except:
            self.env.close()
            self.demand_generator.reset()
            for vehicle in self.vehicles.values():
                vehicle.destroy()
            self.env = SumoEnvironment("../../../environments/sumo/networks/single_intersection/intersection.sumocfg",
                                       demand_generator=self.demand_generator, time_step=0.1, gui=self.gui,
                                       warnings=False)
            self.env.step()  # Load all the vehicles in
            self.vehicles = {vehicle_id: RLVehicle(vehicle_id, self.env, "intersection") for
                             vehicle_id in self.env.vehicles.get_ids()}
            for vehicle in self.vehicles.values():
                vehicle.step()  # So they all broadcast their details, ready for the observations

        return {vehicle_id: vehicle.get_observation() for vehicle_id, vehicle in self.vehicles.items()}

    def step(self, action_dict: MultiAgentDict) \
            -> Tuple[MultiAgentDict, MultiAgentDict, MultiAgentDict, MultiAgentDict]:

        for v, action in action_dict.items():
            self.vehicles[v].apply_action(action)

        for _ in range(self.env_steps_per_step):
            self.env.step()

            if len(self.env.get_removed_vehicles()) > 0:
                obs = {vehicle: self.observation_space.sample() for vehicle in self.vehicles}
                rewards = {vehicle: -20 for vehicle in self.vehicles}
                dones = {vehicle: True for vehicle in self.vehicles}
                infos = {vehicle: {} for vehicle in self.vehicles}
                dones["__all__"] = True
                return obs, rewards, dones, infos

        for vehicle in self.vehicles.values():
            vehicle.step()  # So they all broadcast their details, ready for the observations

        obs = {vehicle_id: vehicle.get_observation() for vehicle_id, vehicle in self.vehicles.items()}
        rewards = {vehicle_id: vehicle.get_reward() for vehicle_id, vehicle in self.vehicles.items()}
        dones = {vehicle_id: False for vehicle_id, vehicle in self.vehicles.items()}
        infos = {vehicle_id: {} for vehicle_id, vehicle in self.vehicles.items()}
        dones["__all__"] = False

        return obs, rewards, dones, infos
