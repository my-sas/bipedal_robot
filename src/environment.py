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

        # action space includes efforts of six joints
        self.action_space = spaces.Box(
            low=np.array([
                -1., -1., -1., -1., -1.,
                -1., -1., -1., -1., -1.
            ]),
            high=np.array([
                1., 1., 1., 1., 1.,
                1., 1., 1., 1., 1.
            ]),
            shape=(10,), dtype=np.float32)

        self.observation_space = spaces.Box(
            low=np.array([
                -0.26, -1.57, 0, -1.05, -0.52,  # left leg low joint positions
                -1.5, -1.5, -1.5, -1.5, -1.5,  # left leg low joint velocities
                -0.26, -1.57, 0, -1.05, -0.52,  # right leg low joint positions
                -1.5, -1.5, -1.5, -1.5, -1.5,  # right low leg joint velocities
                -1.1, -1.1, -1.1,  # root link low velocity
                -1., -1., -1., -1., -1., -1., -1., -1., -1., -1.  # low previous action
            ]),
            high=np.array([
                0.78, 1.57, 1.57, 0.52, 0.52,  # left leg high joint positions
                1.5, 1.5, 1.5, 1.5, 1.5,  # left leg high joint velocities
                0.78, 1.57, 1.57, 0.52, 0.52,  # right leg high joint positions
                1.5, 1.5, 1.5, 1.5, 1.5,  # right leg high joint velocities
                1.1, 1.1, 1.1,  # root link high velocity
                1., 1., 1., 1., 1., 1., 1., 1., 1., 1.  # high previous action
            ]),
            shape=(33,), dtype=np.float32)
        self.prev_action = np.zeros(10)

        # other environment parameters
        self.rate = rospy.Rate(60)  # rate of actions
        self.step_n = 0  # step counter
        self.max_step = 10000  # max length of episode
        self.min_height = 0.11
        self.pose = pose  # initial coordinates

        # initialize ros interfaces
        self.spawner = Spawner(name)
        self.spawner.spawn(self.pose)
        time.sleep(2)

        self.joint_listener = JointListener(name)
        self.link_listener = LinkListener(name)
        self.effort_publisher = EffortPublisher(name)
        self.velocity_listener = VelocityListener(name)
        # self.contact_listener = ContactListener(name)
        self.reloader = Reloader(name, self.pose)

    def reward_func(self, v, h, efforts, step_n):
        # print(f"{h*0.9:.3f} {np.abs(efforts).sum()*0.016:.3f} {v*0.5:.3f}")
        return -(1.1 + h * 0.9)
        # return -(v*0.5 + 1.1 + h*0.9 - np.abs(efforts).sum()*0.016)

    def is_done(self, h):
        return (self.step_n > self.max_step) or (h < self.min_height)

    def step(self, action):
        self.rate.sleep()
        self.step_n += 1

        # do action
        self.effort_publisher.send(action * 10)

        # get observation data
        joint_data = self.joint_listener.get_data()
        orientation, coordinates = self.link_listener.get_data("dummy")  # body coordinates
        velocity_data = self.velocity_listener.get_data()
        # contact_data = self.contact_listener.get_data()  # no contact data yet

        self.state = np.concatenate((joint_data, velocity_data, self.prev_action))
        reward = self.reward_func(velocity_data[0], coordinates[-1], action, self.step_n)  # forward speed, body height
        done = self.is_done(coordinates[-1])  # body height
        info = {}

        # update previous action
        self.prev_action = action

        return self.state, reward, done, False, info

    def reset(self, seed=None, options=None):
        print('Episode done')
        self.effort_publisher.send([0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

        self.step_n = 0
        self.prev_action = np.zeros(10)
        reset_simulation()
        self.effort_publisher.send([0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
        # self.reloader.reload()

        joint_data = self.joint_listener.get_data()
        orientation, coordinates = self.link_listener.get_data("dummy")
        velocity_data = self.velocity_listener.get_data()
        state = np.concatenate((joint_data, velocity_data, self.prev_action))
        return state, {}

    def close(self):
        # Закрытие соединений и очистка ресурсов
        pass
