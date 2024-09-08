import gymnasium as gym
from stable_baselines3 import DDPG
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.noise import NormalActionNoise
from environment import Environment
import numpy as np
import time

NUM_PROC = 4
BASE_NAME = "robot"

INIT_POSE = [0, 0, 2.8]
SPACE = 10


def make_env(name, pose):

    def _init() -> gym.Env:
        env = Environment(name, pose)
        env.reset()
        return env

    time.sleep(2)
    return _init

if __name__ == '__main__':
    names = [BASE_NAME + str(i) for i in range(NUM_PROC)]
    poses = []
    pose = INIT_POSE.copy()
    for i in range(NUM_PROC):
        poses.append(pose.copy())
        pose[1] += SPACE

    env = SubprocVecEnv([make_env(name, pose) for name, pose in zip(names, poses)])



    model = DDPG("MlpPolicy", env, verbose=1, learning_rate=0.003, )
    model.learn(total_timesteps=100000)
