import gym
from gym import spaces
import numpy as np
import time
from ros_bridge import JointListener, LinkListener, EffortPublisher, VelocityListener, reset_simulation


class Environment(gym.Env):
    def __init__(self):
        super(Environment, self).__init__()
        self.action_space = spaces.Box(low=-1., high=1., shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(low=-1., high=1., shape=(8+3,), dtype=np.float32)

        self.joint_listener = JointListener()
        self.link_listener = LinkListener()
        self.effort_publisher = EffortPublisher()
        self.velocity_listener = VelocityListener()

    def reward_func(self, v, h):
        return v + 0.05 - (h * 10) ** 2

    def is_done(self, h):
        return h < 0.8

    def step(self, action):
        time.sleep(0.1)

        # do action
        self.effort_publisher.send(action)

        # get observation data
        joint_data = self.joint_listener.get_data()
        link_data = self.link_listener.get_data('robot::dummy') # body coordinates
        velocity_data = self.velocity_listener.get_data()

        observation = np.array(joint_data + link_data)
        reward = self.reward_func(velocity_data, link_data[-1]) # forward speed, body height
        done = self.is_done(link_data[-1]) # body height
        info = {}
        return observation, reward, done, info

    def reset(self):
        reset_simulation()
        return  # Пример начального состояния

    def close(self):
        # Закрытие соединений и очистка ресурсов
        pass
