import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from environment import Environment
import numpy as np
import rospy
import time

NUM_PROC = 2
BASE_NAME = "robot"

INIT_POSE = [0, 0, 1.7]
SPACE = 10

# n_actions = 10
# action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=0.1, theta=0.15)
# policy_kwargs = dict(
#     net_arch=dict(pi=[128, 64, 32], qf=[128, 64, 32])
# )

def make_env(name, pose):

    def _init() -> gym.Env:
        env = Environment(name, pose)
        env.reset()
        return env

    time.sleep(2)
    return _init


if __name__ == '__main__':
    rospy.init_node("train_env")
    names = [BASE_NAME + str(i) for i in range(NUM_PROC)]
    poses = []
    pose = INIT_POSE.copy()
    for i in range(NUM_PROC):
        poses.append(pose.copy())
        pose[1] += SPACE

    env = DummyVecEnv([make_env(name, pose) for name, pose in zip(names, poses)])

    model = PPO("MlpPolicy", env, learning_rate=0.01, device="cuda")
    model.learn(total_timesteps=1000000)
