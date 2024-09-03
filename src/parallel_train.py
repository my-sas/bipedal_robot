from stable_baselines3 import DDPG
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.noise import NormalActionNoise
from environment import Environment
import numpy as np

NUM_PROC = 4
BASE_NAME = "robot"


def make_env(name):
    """
    Utility function for multiprocessed env.

    :param env_id: (str) the environment ID
    :param num_env: (int) the number of environment you wish to have in subprocesses
    :param seed: (int) the inital seed for RNG
    :param rank: (int) index of the subprocess
    :return: (Callable)
    """

    def _init() -> gym.Env:
        env = Environment(name)
        env.reset()
        return env

    return _init


env = SubprocVecEnv([make_env(BASE_NAME + str(i)) for i in range(NUM_PROC)])
model = DDPG("MlpPolicy", env, verbose=1, learning_rate=0.003, )
model.learn(total_timesteps=1000000)