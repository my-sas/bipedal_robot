"""This module
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time
import rospy
from ros_bridge import Spawner, JointListener, LinkListener, EffortPublisher, VelocityListener, Reloader, reset_simulation


class Environment(gym.Env):
    def __init__(self, name, pose):
        super(Environment, self).__init__()

        # initialize node and set namespace
        self.name = name
        rospy.init_node(f"{name}_env", anonymous=True, disable_signals=False)

        # define action and observation spaces
        self.state = None  # current robot state
        self.action_space = spaces.Box(
            low=np.array([-50., -50., -30., -30.]),
            high=np.array([50., 50., 30., 30.]),
            shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=-50., high=50.,
            shape=(12+24,), dtype=np.float32)
        self.prev_actions = [np.zeros(4)] * 6

        # other environment parameters
        self.rate = rospy.Rate(60)  # rate of actions
        self.step_n = 0  # step counter
        self.max_step = 10000  # max length of episode
        self.min_height = 0.12
        self.pose = pose  # initial coordinates

        # initialize ros interfaces
        self.spawner = Spawner(name)
        self.spawner.spawn(self.pose)
        time.sleep(5)

        self.joint_listener = JointListener(name)
        self.link_listener = LinkListener(name)
        self.effort_publisher = EffortPublisher(name)
        self.velocity_listener = VelocityListener(name)
        # self.contact_listener = ContactListener(name)
        self.reloader = Reloader(name, self.pose)

    def reward_func(self, v, h, efforts, step_n):
        return v*0.3 + 0.9 + h*1.1

    def is_done(self, h):
        return (self.step_n > self.max_step) or (h < self.min_height)

    def step(self, action):
        self.rate.sleep()
        self.step_n += 1

        # do action
        self.effort_publisher.send(action)

        # get observation data
        joint_data = self.joint_listener.get_data()
        orientation, coordinates = self.link_listener.get_data("dummy")  # body coordinates
        velocity_data = self.velocity_listener.get_data()
        # contact_data = self.contact_listener.get_data()  # no contact data yet

        self.state = np.concatenate([np.array(joint_data + orientation),
                                     np.concatenate(self.prev_actions)])
        reward = self.reward_func(velocity_data, coordinates[-1], action, self.step_n)  # forward speed, body height
        done = self.is_done(coordinates[-1])  # body height
        info = {}

        self.prev_actions.pop()
        self.prev_actions.append(action)

        return self.state, reward, done, False, info

    def reset(self, seed=None, options=None):
        print('Episode done')
        self.effort_publisher.send([0., 0., 0., 0.])

        self.step_n = 0
        self.prev_actions = [np.zeros(4)] * 6
        reset_simulation()
        self.effort_publisher.send([0., 0., 0., 0.])
        # self.reloader.reload()

        joint_data = self.joint_listener.get_data()
        orientation, coordinates = self.link_listener.get_data("dummy")
        return np.concatenate([np.array(joint_data + orientation), np.concatenate(self.prev_actions)]), {}

    def close(self):
        # Закрытие соединений и очистка ресурсов
        pass
