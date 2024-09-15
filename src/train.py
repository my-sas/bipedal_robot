from stable_baselines3 import DDPG
from stable_baselines3.common.noise import NormalActionNoise
from environment import Environment
import numpy as np

env = Environment("robot", [0, 0, 0.3])

n_actions = 10
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.066*np.ones(n_actions))
model = DDPG("MlpPolicy", env, verbose=1, learning_rate=0.0003, action_noise=action_noise)
# model = DDPG.load("../models/weights1", env=env)

model.learn(total_timesteps=1000000)
# model.save("../models/weights1")
