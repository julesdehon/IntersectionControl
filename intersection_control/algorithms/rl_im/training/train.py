import numpy as np
import ray
import time
from datetime import datetime
from ray.rllib.agents import ppo
from ray.tune.registry import register_env

from intersection_control.algorithms.rl_im.constants import RLMode
from intersection_control.algorithms.rl_im.rl_vehicle import RLVehicle
from intersection_control.environments import SumoEnvironment
from intersection_control.environments.sumo import RandomDemandGenerator
from intersection_control.environments.sumo.sumo_environment import ControlType
from single_intersection_sumo_env import MultiAgentSumoEnv

CHECKPOINT_EVERY = 30
N_ITER = 30
CHECKPOINT_ROOT = "./checkpoints"


def train():
    ray.init()

    config = ppo.DEFAULT_CONFIG.copy()

    def env_creator(env_config):
        return MultiAgentSumoEnv(env_config, gui=False, env_steps_per_step=5)

    register_env("single-intersection", env_creator)

    trainer = ppo.PPOTrainer(env="single-intersection", config={
        **config,
        "batch_mode": "complete_episodes",
        "disable_env_checking": True
    })

    start = time.time()
    out = open("out.txt", "a")

    step = 0
    while time.time() - start < 60 * 60 * 3:
        result = trainer.train()

        if step % CHECKPOINT_EVERY == 0:
            trainer.save(CHECKPOINT_ROOT)

        summary = f"[{datetime.now().strftime('%H:%M:%S')}] {step + 1}: {result['episode_reward_min']}/" \
                  f"{result['episode_reward_mean']}/{result['episode_reward_max']} " \
                  f"len {result['episode_len_mean']}"
        print(summary)
        out.write(summary)
        step += 1
    out.close()


def evaluate():
    np.set_printoptions(suppress=True)
    ray.init()

    config = ppo.DEFAULT_CONFIG.copy()

    def env_creator(env_config):
        return MultiAgentSumoEnv(env_config, gui=False, env_steps_per_step=2)

    register_env("single-intersection", env_creator)

    trainer = ppo.PPOTrainer(env="single-intersection", config={
        **config,
        "batch_mode": "complete_episodes",
        "disable_env_checking": True
    })

    trainer.restore("./checkpoints/checkpoint_000151/checkpoint-151")

    demand_generator = RandomDemandGenerator({
        "NE": 1, "NS": 1, "NW": 1,
        "EN": 1, "ES": 1, "EW": 1,
        "SN": 1, "SE": 1, "SW": 1,
        "WN": 1, "WE": 1, "WS": 1
    }, 0.05, control_type=ControlType.MANUAL)
    env = SumoEnvironment("../../../environments/sumo/networks/single_intersection/intersection.sumocfg",
                          demand_generator=demand_generator, time_step=0.05, gui=True)
    vehicles = {RLVehicle(vehicle_id, env, env.intersections.get_ids()[0], RLMode.EVALUATE, trainer) for vehicle_id in
                env.vehicles.get_ids()}

    step_count = 360000  # 1 hour
    for _ in range(step_count):
        env.step()
        removed_vehicles = {v for v in vehicles if v.get_id() in env.get_removed_vehicles()}
        for v in removed_vehicles:
            v.destroy()
        new_vehicles = {RLVehicle(vehicle_id, env, env.intersections.get_ids()[0], RLMode.EVALUATE, trainer) for
                        vehicle_id in env.get_added_vehicles()}
        vehicles = (vehicles - removed_vehicles).union(new_vehicles)
        for vehicle in vehicles:
            vehicle.step()


if __name__ == "__main__":
    evaluate()
