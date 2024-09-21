"""This module
"""

import time
# import logging
import numpy as np
import tf.transformations as tft

import rospy
from ros_bridge import *

import gymnasium as gym
from gymnasium import spaces

# logging.basicConfig(filename='../logs/env.log', filemode='a', level=logging.INFO)


class Environment(gym.Env):
    def __init__(self, name, pose):
        super(Environment, self).__init__()

        # initialize node and set namespace
        self.name = name
        rospy.init_node(f"{name}_env")

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
                -1., -1., -1., -1.,  # orientation
                -1.57, -0.26, 0, -1.05, -0.52,  # left leg low joint positions
                -1.57, -0.78, 0, -1.05, -0.52,  # right leg low joint positions
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
                1., 1., 1., 1.,  # orientation
                0.26, 0.78, 1.57, 0.52, 0.52,  # left leg high joint positions
                0.26, 0.26, 1.57, 0.52, 0.52,  # right leg high joint positions
                1.5, 1.5, 1.5, 1.5, 1.5,  # left leg high joint velocities
                1.5, 1.5, 1.5, 1.5, 1.5,  # right leg high joint velocities
                2.1, 2.1, 2.1,  # root link high velocity
                1., 1., 1., 1., 1., 1., 1., 1., 1., 1.,  # high previous action
                float("inf"), float("inf"), float("inf"),
                float("inf"), float("inf"), float("inf"),
                float("inf"), float("inf"), float("inf"),
                float("inf"), float("inf"), float("inf")
            ]),
            shape=(49,), dtype=np.float32)
        self.prev_action = np.zeros(10)

        # other environment parameters
        self.rate = rospy.Rate(50)  # rate of actions
        self.step_n = 0  # step counter
        self.time = time.time()
        self.max_step = 10000  # max length of episode
        self.min_height = 0.6
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

    def reward_func(self, v_x, v_y, h, pitch, efforts, step_n):
        # print(f"{np.sqrt((1.5 - h)**2):.3f} {np.sqrt(pitch**2):.3f} {np.sqrt(efforts**2).sum()/20:.3f}")
        return (8.0 - np.sqrt((1.48 - h)**2) - np.sqrt(pitch**2) - np.sqrt(efforts**2).sum()/30)*0.3

    def is_done(self, h):
        return (self.step_n > self.max_step) or (h < self.min_height)

    def step(self, action):
        self.rate.sleep()
        self.step_n += 1

        # do action
        self.effort_publisher.send(action * np.array([20., 20., 20., 10., 10., 20., 20., 20., 10., 10.]))

        # get observation data
        joint_data = self.joint_listener.get_data()
        orientation, coordinates = self.link_listener.get_data("dummy")  # body coordinates
        velocity_data = self.velocity_listener.get_data()
        left_contact_data = self.left_foot_contact_listener.get_data() / 1000
        right_contact_data = self.right_foot_contact_listener.get_data() / 1000

        self.state = np.concatenate((
            orientation,
            joint_data, velocity_data, self.prev_action,
            left_contact_data, right_contact_data
        ))

        orientation_euler = tft.euler_from_quaternion(orientation)
        # print(orientation_euler, coordinates)

        reward = self.reward_func(velocity_data[0], velocity_data[1], coordinates[2], orientation_euler[1], action, self.step_n)  # forward speed, body height
        done = self.is_done(coordinates[-1])  # body height
        info = {}

        # update previous action
        self.prev_action = action
        return self.state, reward, done, False, info

    def reset(self, seed=None, options=None):
        cur_time = time.time()

        orientation, coordinates = self.link_listener.get_data("dummy")
        print(
            f"Episode done, " +
            f"Steps: {self.step_n}, " +
            f"Time: {cur_time - self.time:.3f}, " +
            f"Hz: {self.step_n/(cur_time - self.time):.3f}, " +
            f"X_dist: {coordinates[0]:.3f}, " +
            f"Y_dist: {coordinates[1]:.3f}"
        )

        self.effort_publisher.send(np.zeros(10))

        self.step_n = 0
        self.prev_action = np.zeros(10)

        # self.rate = rospy.Rate(45)

        # reset_simulation()
        self.reloader.reload()

        self.effort_publisher.send(np.zeros(10))

        joint_data = self.joint_listener.get_data()
        orientation, coordinates = self.link_listener.get_data("dummy")
        velocity_data = self.velocity_listener.get_data()
        left_contact_data = self.left_foot_contact_listener.get_data() / 1000
        right_contact_data = self.right_foot_contact_listener.get_data() / 1000

        state = np.concatenate((
            orientation,
            joint_data, velocity_data, self.prev_action,
            left_contact_data, right_contact_data
        ))

        self.time = time.time()
        return state, {}

    def close(self):
        # Закрытие соединений и очистка ресурсов
        pass
