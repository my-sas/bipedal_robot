from stable_baselines3 import DDPG
from environment import Environment

env = Environment()
model = DDPG("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=1000)
