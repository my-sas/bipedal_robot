"""This module
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time
import rospy
from ros_bridge import Spawner, JointListener, LinkListener, EffortPublisher, VelocityListener, ContactListener, Reloader, reset_simulation


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
                -0.78, -1.57, 0, -1.05, -0.52,  # right leg low joint positions
                -1.5, -1.5, -1.5, -1.5, -1.5,  # left leg low joint velocities
                -1.5, -1.5, -1.5, -1.5, -1.5,  # right low leg joint velocities
                -2.1, -2.1, -2.1,  # root link low velocity
                -1., -1., -1., -1., -1., -1., -1., -1., -1., -1.,  # low previous action
                float("-inf"), float("-inf"), float("-inf"),
                float("-inf"), float("-inf"), float("-inf"),
                float("-inf"), float("-inf"), float("-inf"),
                float("-inf"), float("-inf"), float("-inf")

            ]),
            high=np.array([
                0.78, 1.57, 1.57, 0.52, 0.52,  # left leg high joint positions
                0.26, 1.57, 1.57, 0.52, 0.52,  # right leg high joint positions
                1.5, 1.5, 1.5, 1.5, 1.5,  # left leg high joint velocities
                1.5, 1.5, 1.5, 1.5, 1.5,  # right leg high joint velocities
                2.1, 2.1, 2.1,  # root link high velocity
                1., 1., 1., 1., 1., 1., 1., 1., 1., 1.,  # high previous action
                float("inf"), float("inf"), float("inf"),
                float("inf"), float("inf"), float("inf"),
                float("inf"), float("inf"), float("inf"),
                float("inf"), float("inf"), float("inf")
            ]),
            shape=(45,), dtype=np.float32)
        self.prev_action = np.zeros(10)

        # other environment parameters
        self.rate = rospy.Rate(45)  # rate of actions
        self.step_n = 0  # step counter
        self.max_step = 10000  # max length of episode
        self.min_height = 0.5
        self.pose = pose  # initial coordinates

        # initialize ros interfaces
        self.spawner = Spawner(name)
        self.spawner.spawn(self.pose)
        rospy.sleep(5)

        self.joint_listener = JointListener(name)
        self.link_listener = LinkListener(name)
        self.effort_publisher = EffortPublisher(name)
        self.velocity_listener = VelocityListener(name)
        self.left_foot_contact_listener = ContactListener(name, "left_foot")
        self.right_foot_contact_listener = ContactListener(name, "right_foot")
        self.reloader = Reloader(name, pose)

    def reward_func(self, v_x, v_y, h, efforts, step_n):
        # print(f"{h*0.9:.3f} {np.abs(efforts).sum()*0.016:.3f} {v*0.5:.3f}")
        # print(f"{v_x*0.4:.3f} {h*0.9:.3f} {- np.sqrt((efforts**2).sum())/30:.3f} {- np.sqrt(v_y**2)*0.5:.3f}")
        return v_x*0.4 + 1.1 + h*0.9 - np.sqrt((efforts**2).sum())/30 - np.sqrt(v_y**2)*0.5

    def is_done(self, h):
        return (self.step_n > self.max_step) or (h < self.min_height)

    def step(self, action):

        # do action
        self.effort_publisher.send(action * 20)

        # get observation data
        joint_data = self.joint_listener.get_data()
        orientation, coordinates = self.link_listener.get_data("dummy")  # body coordinates
        velocity_data = self.velocity_listener.get_data()
        left_contact_data = self.left_foot_contact_listener.get_data() / 1000
        right_contact_data = self.right_foot_contact_listener.get_data() / 1000

        # print(f"{coordinates[-1]:.3f} {velocity_data[0]:.3f} {np.abs(action).sum():.3f}")
        # print(f"{sum(left_contact_data[:3]):.3f}")

        self.state = np.concatenate((
            joint_data, velocity_data, self.prev_action,
            left_contact_data, right_contact_data
        ))
        reward = self.reward_func(velocity_data[0], velocity_data[1], coordinates[-1], action, self.step_n)  # forward speed, body height
        done = self.is_done(coordinates[-1])  # body height
        info = {}

        # update previous action
        self.prev_action = action

        self.step_n += 1
        self.rate.sleep()
        return self.state, reward, done, False, info

    def reset(self, seed=None, options=None):
        print('Episode done')
        self.effort_publisher.send(np.zeros(10))

        self.step_n = 0
        self.prev_action = np.zeros(10)

        # reset_simulation()
        self.reloader.reload()

        self.effort_publisher.send(np.zeros(10))

        joint_data = self.joint_listener.get_data()
        orientation, coordinates = self.link_listener.get_data("dummy")
        velocity_data = self.velocity_listener.get_data()
        left_contact_data = self.left_foot_contact_listener.get_data() / 1000
        right_contact_data = self.right_foot_contact_listener.get_data() / 1000
        state = np.concatenate((
            joint_data, velocity_data, self.prev_action,
            left_contact_data, right_contact_data
        ))
        return state, {}

    def close(self):
        # Закрытие соединений и очистка ресурсов
        pass
