import ray
from ray.rllib.agents.ppo import PPOTrainer
from ray.tune.logger import pretty_print
from sumo_env import MultiAgentSumoEnv


def main():
    ray.init()

    trainer = PPOTrainer(env=MultiAgentSumoEnv, config={
        "env_config": {
            "simulation_time": 60 * 5  # 5 minutes
        }
    })

    print(pretty_print(trainer.train()))


if __name__ == "__main__":
    main()
