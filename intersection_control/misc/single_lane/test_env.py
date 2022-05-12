import random
from typing import Tuple
import ray
from ray.rllib.agents import ppo
from ray.rllib.agents import pg
from ray.rllib.env.multi_agent_env import MultiAgentEnv
from ray.rllib.utils.typing import MultiAgentDict
import gym.spaces as spaces
from ray.tune import register_env

N_ITER = 30


class ReproducedEnv(MultiAgentEnv):
    def __init__(self, _):
        super().__init__()
        self._agent_ids = {"0"}
        self._all_ever_agents = {"0"}
        self.max_steps = random.randint(50, 100)
        self.steps = 0

        self.observation_space = spaces.Discrete(2)
        self.action_space = spaces.Discrete(2)

        # #####
        self.empty_observations = 0

    def reset(self) -> MultiAgentDict:
        self._agent_ids = {"0"}
        self._all_ever_agents = {"0"}
        self.max_steps = random.randint(50, 100)
        self.steps = 0

        # ####
        self.empty_observations = 0
        return {}

    def step(self, action_dict: MultiAgentDict) \
            -> Tuple[MultiAgentDict, MultiAgentDict, MultiAgentDict, MultiAgentDict]:

        obs, rewards, dones, infos = {}, {}, {}, {}

        # 10% chance of adding a new agent
        # if random.random() < 0.1:
        #     agent_id = len(self._agent_ids)
        #     self._agent_ids.add(f"{agent_id}")
        #     self._all_ever_agents.add(f"{agent_id}")

        # 10% chance of removing a random agent
        # if random.random() < 0.1 and len(self._agent_ids) > 1:
        #     removed_agent = self._agent_ids.pop()
        #     obs[removed_agent] = self.observation_space.sample()
        #     rewards[removed_agent] = 0
        #     dones[removed_agent] = True
        #     infos[removed_agent] = {}

        obs.update({agent: self.observation_space.sample() for agent in self._agent_ids})
        rewards.update({agent: random.uniform(-100, 100) for agent in obs})
        dones.update({agent: False for agent in obs})
        infos.update({agent: {} for agent in obs})

        self.steps += 1
        if self.steps >= self.max_steps:
            obs = {agent: self.observation_space.sample() for agent in self._all_ever_agents}
            rewards = {agent: 0 for agent in self._all_ever_agents}
            dones = {agent: True for agent in self._all_ever_agents}
            infos = {agent: {} for agent in self._all_ever_agents}
            dones["__all__"] = True
        else:
            dones["__all__"] = False

        if len(obs) == 0:
            self.empty_observations += 1

        return obs, rewards, dones, infos


def main():
    ray.init()

    config = ppo.DEFAULT_CONFIG.copy()
    config = pg.DEFAULT_CONFIG.copy()

    # def env_creator(args):
    #     return PettingZooEnv(waterworld_v3.env())
    def env_creator(args):
        return ReproducedEnv({})

    register_env("waterworld", env_creator)

    trainer = pg.PGTrainer(env="waterworld", config={
        **config,
        "horizon": 200
    })

    for n in range(N_ITER):
        result = trainer.train()
        print(f"{n + 1}: {result['episode_reward_mean']}")


def main2():
    env = ReproducedEnv({})
    obs = env.reset()
    dones = {"__all__": False}
    empty_observations = 0
    i = 0
    while not dones["__all__"]:
        i += 1
        obs, rewards, dones, infos = env.step({agent: env.observation_space.sample() for agent in obs})
        if len(obs) == 0:
            empty_observations += 1
    print(f"Iterations: {i}. Empty obs: {empty_observations}")


if __name__ == "__main__":
    main()
