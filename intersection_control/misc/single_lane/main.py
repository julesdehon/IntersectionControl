import ray
from ray.rllib.agents import ppo
from single_road_env import SingleRoadEnv

N_ITER = 30
CHECKPOINT_ROOT = "./checkpoints"


def train():
    ray.init()

    config = ppo.DEFAULT_CONFIG.copy()

    trainer = ppo.PPOTrainer(env=SingleRoadEnv, config={
        **config,
        "batch_mode": "complete_episodes",
        "disable_env_checking": True
    })

    for n in range(N_ITER):
        result = trainer.train()
        file_name = trainer.save(CHECKPOINT_ROOT)

        print(f"{n + 1}: {result['episode_reward_min']}/{result['episode_reward_mean']}/{result['episode_reward_max']} "
              f"len {result['episode_len_mean']} saved {file_name}")


def simulate_using_trained_trainer():
    ray.init()

    config = ppo.DEFAULT_CONFIG.copy()

    trainer = ppo.PPOTrainer(env=SingleRoadEnv, config={
        **config,
        "batch_mode": "complete_episodes",
        "disable_env_checking": True
    })

    trainer.restore("./checkpoints/checkpoint_000030/checkpoint-30")

    env = SingleRoadEnv({}, gui=False)
    obs = env.reset()
    dones = {"__all__": False}
    while not dones["__all__"]:
        actions = trainer.compute_actions(obs)
        print(actions)
        obs, rewards, dones, info = env.step(actions)


if __name__ == "__main__":
    train()
