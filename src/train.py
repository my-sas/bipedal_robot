import numpy as np
from environment import Environment
from stable_baselines3 import DDPG, PPO
from stable_baselines3.common.noise import OrnsteinUhlenbeckActionNoise

env = Environment("robot", [0, 0, 1.7], init_node=True)

n_actions = 10
action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(n_actions), sigma=0.1, theta=0.15)
policy_kwargs = dict(
    net_arch=dict(pi=[128, 64, 32], qf=[128, 64, 32])
)
#
# model = DDPG(
#     "MlpPolicy", env,
#     policy_kwargs=policy_kwargs,gamma=1,
#     action_noise=action_noise, device="cuda"
# )

# model = PPO(
#     "MlpPolicy", env,
#     verbose=1, learning_rate=0.001,
# )

# print(model.policy.actor)
# print(model.policy.critic)
model = DDPG.load("../models/weights4.zip", env=env, action_noise=action_noise, device="cuda")

model.learn(total_timesteps=100000)
model.save("../models/weights4")
