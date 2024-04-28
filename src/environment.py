import gymnasium as gym
from gymnasium import spaces
import numpy as np
import time
import rospy
from ros_bridge import JointListener, LinkListener, EffortPublisher, VelocityListener, Resetter, reset_simulation


# class Environment(gym.Env):
#     def __init__(self):
#         super(Environment, self).__init__()
#         self.action_space = spaces.Box(
#             low=1., high=9., shape=(4,), dtype=np.float32
#         )
#         self.observation_space = spaces.Box(
#             low=-20., high=20., shape=(8+3,), dtype=np.float32
#         )
#
#         self.rate = rospy.Rate(30)
#
#         self.joint_listener = JointListener()
#         self.link_listener = LinkListener()
#         self.effort_publisher = EffortPublisher()
#         self.velocity_listener = VelocityListener()
#         self.resetter = Resetter()
#
#     def reward_func(self, v, h):
#         return v + 0.05 - (h ** 2)
#
#     def is_done(self, h):
#         return h < 0.9
#
#     def step(self, action):
#         self.rate.sleep()
#         # rospy.sleep(15.0/60.0)
#
#         # do action
#         self.effort_publisher.send(action)
#
#         # get observation data
#         joint_data = self.joint_listener.get_data()
#         link_data = self.link_listener.get_data('robot::dummy') # body coordinates
#         velocity_data = self.velocity_listener.get_data()
#
#         observation = np.array(joint_data + link_data)
#         reward = self.reward_func(velocity_data, link_data[-1]) # forward speed, body height
#         done = self.is_done(link_data[-1]) # body height
#         info = {}
#         # print(reward, action, velocity_data, link_data[-1])
#         print(observation)
#         return observation, reward, done, info
#
#     def reset(self):
#         reset_simulation()
#         # self.resetter.reset()
#         return  # Пример начального состояния
#
#     def close(self):
#         # Закрытие соединений и очистка ресурсов
#         pass



class Environment(gym.Env):
    def __init__(self):
        super(Environment, self).__init__()
        self.state = None
        self.action_space = spaces.Box(
            low=-9., high=9., shape=(4,), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-20., high=20., shape=(8+3,), dtype=np.float32
        )

        self.rate = rospy.Rate(30)

        self.joint_listener = JointListener()
        self.link_listener = LinkListener()
        self.effort_publisher = EffortPublisher()
        self.velocity_listener = VelocityListener()
        self.resetter = Resetter()

    def reward_func(self, v, h):
        return v + 0.05 - (h ** 2)

    def is_done(self, h):
        return h < 0.9

    def step(self, action):
        self.rate.sleep()

        # do action
        self.effort_publisher.send(action)

        # get observation data
        joint_data = self.joint_listener.get_data()
        link_data = self.link_listener.get_data('robot::dummy') # body coordinates
        velocity_data = self.velocity_listener.get_data()

        self.state = np.array(joint_data + link_data)
        reward = self.reward_func(velocity_data, link_data[-1]) # forward speed, body height
        done = self.is_done(link_data[-1]) # body height
        info = {}
        # print(reward, action, velocity_data, link_data[-1])
        return self.state, reward, done, False, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        reset_simulation()

        joint_data = self.joint_listener.get_data()
        link_data = self.link_listener.get_data('robot::dummy')
        return np.array(joint_data + link_data), {}

    def close(self):
        # Закрытие соединений и очистка ресурсов
        pass
