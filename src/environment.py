"""This module
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time
import rospy
from ros_bridge import Spawner, JointListener, LinkListener, EffortPublisher, VelocityListener, Reloader, ContactListener


class Environment(gym.Env):
    def __init__(self, name):
        super(Environment, self).__init__()
        self.name = name
        self.state = None
        self.action_space = spaces.Box(
            low=np.array([-50., -50., -30., -30.]),
            high=np.array([50., 50., 30., 30.]),
            shape=(4,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-50., high=50., shape=(12+24,), dtype=np.float32
        )

        self.rate = rospy.Rate(60)
        self.step_n = 0
        self.prev_actions = [np.zeros(4)]*6

        self.spawner =
        self.joint_listener = JointListener()
        self.link_listener = LinkListener()
        self.effort_publisher = EffortPublisher()
        self.velocity_listener = VelocityListener()
        self.contact_listener = ContactListener()
        self.resetter = Reloader()

    def reward_func(self, v, h, efforts, step_n):
        return v*4 + 0.1*step_n  # + h*3 - np.sum(abs(efforts))*0.1

    def is_done(self, h):
        return (self.step_n > 1000) or (h < 0.65)

    def step(self, action):
        self.rate.sleep()
        self.step_n += 1

        # do action
        self.effort_publisher.send(action)

        # get observation data
        joint_data = self.joint_listener.get_data()
        orientation, coordinates = self.link_listener.get_data("dummy") # body coordinates
        velocity_data = self.velocity_listener.get_data()
        # contact_data = self.contact_listener.get_data() # no contact data yet

        self.state = np.concatenate([np.array(joint_data + orientation),
                                     np.concatenate(self.prev_actions)])
        reward = self.reward_func(velocity_data, coordinates[-1], action, self.step_n) # forward speed, body height
        done = self.is_done(coordinates[-1]) # body height
        info = {}

        self.prev_actions.pop()
        self.prev_actions.append(action)

        # print(f'v: {velocity_data*4}, h: {coordinates[-1]}, efforts: {-np.sum(abs(action))*0.1}, step_n: {self.step_n*0.05}')

        return self.state, reward, done, False, info

    def reset(self, seed=None, options=None):
        print('Episode done')
        super().reload(seed=seed, options=options)

        self.step_n = 0
        self.prev_actions = [np.zeros(4)] * 6
        # reset_simulation()
        self.resetter.reload()

        joint_data = self.joint_listener.get_data()
        orientation, coordinates = self.link_listener.get_data('robot::dummy')
        return np.concatenate([np.array(joint_data + orientation), np.concatenate(self.prev_actions)]), {}

    def close(self):
        # Закрытие соединений и очистка ресурсов
        pass
