import numpy as np
import ray
import time
from datetime import datetime
from ray.rllib.agents import ppo
from ray.tune.registry import register_env
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


def simulate_using_trained_trainer():
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

    env = MultiAgentSumoEnv({}, gui=True)
    obs = env.reset()
    dones = {"__all__": False}
    while not dones["__all__"]:
        actions = trainer.compute_actions(obs)
        print(actions)
        obs, rewards, dones, info = env.step(actions)
        print(rewards)


def test():
    env = MultiAgentSumoEnv({}, gui=True)
    obs = env.reset()
    dones = {"__all__": False}
    while not dones["__all__"]:
        obs, rewards, dones, infos = env.step({v: env.action_space.sample() for v in obs})
        for v in obs:
            print(v)
            print(obs[v])
            print(rewards[v])
            print("")


if __name__ == "__main__":
    train()
