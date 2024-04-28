from stable_baselines3 import DDPG
from environment import Environment

env = Environment()
model = DDPG("MlpPolicy", env, verbose=1, learning_rate=0.005, )
model.learn(total_timesteps=5000000)
