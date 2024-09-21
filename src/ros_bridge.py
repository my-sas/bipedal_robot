"""
This module provides interface for interaction with models in gazebo simulation.
You can send model to move joint, get model coordinates, spawn models in simulation, etc.
"""

import time
import numpy as np
import subprocess

import rospy
from gazebo_msgs.srv import GetLinkState
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float64

from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from std_srvs.srv import Empty

# from geometry_msgs.msg import Pose
# from std_srvs.srv import Empty
# from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
# from gazebo_msgs.msg import ModelState
# from gazebo_msgs.srv import GetModelState, GetModelStateRequest
# from controller_manager_msgs.srv import SwitchController


class LinkListener:
    def __init__(self, name):
        rospy.wait_for_service("/gazebo/get_link_state")
        self.service = rospy.ServiceProxy("/gazebo/get_link_state", GetLinkState)
        self.name = name

    def get_data(self, link_name, reference_frame="world"):
        rospy.wait_for_service("/gazebo/get_link_state")
        response = self.service(f"{self.name}::{link_name}", reference_frame)
        if response.success:
            coordinates = response.link_state.pose.position
            orientation = response.link_state.pose.orientation
            return (
                np.array([orientation.x, orientation.y, orientation.z, orientation.w]),
                np.array([coordinates.x, coordinates.y, coordinates.z]))
        else:
            return None


class JointListener:
    def __init__(self, name):
        rospy.Subscriber(f"/{name}/joint_states", JointState, self.callback, queue_size=10)
        self.joint_states = None

    def callback(self, data):
        # rospy.loginfo(f"Joint states updated: {data.position}")
        self.joint_states = np.array(list(data.position) + list(data.velocity))

    def get_data(self):
        return self.joint_states


class EffortPublisher:
    def __init__(self, name):
        self.name = name
        joints = [
            "left_hip_joint1", "left_hip_joint2", "left_knee_joint", "left_ankle_joint1", "left_ankle_joint2",
            "right_hip_joint1", "right_hip_joint2", "right_knee_joint", "right_ankle_joint1", "right_ankle_joint2"
        ]
        joints = [f'/{name}/{join}_effort_controller/command' for join in joints]
        
        self.pub = []
        for joint in joints:
            self.pub.append(rospy.Publisher(joint, Float64, queue_size=10))

    def send(self, efforts: list):
        for i, effort in enumerate(efforts):
            self.pub[i].publish(effort)


class VelocityListener:
    def __init__(self, name):
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        self.name = name
        self.velocity = None

    def callback(self, data):
        robot_index = data.name.index(self.name)
        self.velocity = np.array([
            data.twist[robot_index].linear.x,
            data.twist[robot_index].linear.y,
            data.twist[robot_index].linear.z])

    def get_data(self):
        return self.velocity


class Spawner:
    def __init__(self, name):
        self.name = name
        # self.pose = pose
        # self.command =command = [
        #     "roslaunch", "bipedal_robot",
        #     "spawn.launch", f"name:={name}",
        #     f"pose:=-x {pose[0]} -y {pose[1]} -z {pose[2]}"]

    def spawn(self, pose):
        command = [
            "roslaunch", "bipedal_robot",
            "spawn.launch", f"name:={self.name}",
            f"pose:=-x {pose[0]} -y {pose[1]} -z {pose[2]}"]

        # call spawn.launch file
        subprocess.Popen(command, stdout=subprocess.PIPE)
        time.sleep(2)


class ContactListener:
    def __init__(self, name, link):
        rospy.Subscriber(f"/{name}/{link}_contact", ContactsState, self.callback)
        self.contacts = np.zeros(6)

    def callback(self, data):
        if len(data.states) != 0:
            self.contacts = np.array([
                data.states[0].total_wrench.force.x,
                data.states[0].total_wrench.force.y,
                data.states[0].total_wrench.force.z,
                data.states[0].total_wrench.torque.x,
                data.states[0].total_wrench.torque.y,
                data.states[0].total_wrench.torque.z
            ])
        else:
            return np.zeros(6)

    def get_data(self):
        return self.contacts


class Reloader:
    def __init__(self, name, pose):
        self.joint_name_lst = [
            "left_hip_joint1", "left_hip_joint2", "left_knee_joint", "left_ankle_joint1", "left_ankle_joint2",
            "right_hip_joint1", "right_hip_joint2", "right_knee_joint", "right_ankle_joint1", "right_ankle_joint2"
        ]
        self.starting_pos = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.model_config_proxy = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.model_config_req = SetModelConfigurationRequest()
        self.model_config_req.model_name = name
        self.model_config_req.urdf_param_name = 'robot_description'
        self.model_config_req.joint_names = self.joint_name_lst
        self.model_config_req.joint_positions = self.starting_pos
        self.model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_state_req = SetModelStateRequest()
        self.model_state_req.model_state = ModelState()
        self.model_state_req.model_state.model_name = name
        self.model_state_req.model_state.pose.position.x = pose[0]
        self.model_state_req.model_state.pose.position.y = pose[1]
        self.model_state_req.model_state.pose.position.z = pose[2]
        self.model_state_req.model_state.pose.orientation.x = 0.0
        self.model_state_req.model_state.pose.orientation.y = 0.0
        self.model_state_req.model_state.pose.orientation.z = 0.0
        self.model_state_req.model_state.pose.orientation.w = 0.0
        self.model_state_req.model_state.twist.linear.x = 0.0
        self.model_state_req.model_state.twist.linear.y = 0.0
        self.model_state_req.model_state.twist.linear.z = 0.0
        self.model_state_req.model_state.twist.angular.x = 0.0
        self.model_state_req.model_state.twist.angular.y = 0.0
        self.model_state_req.model_state.twist.angular.z = 0.0
        self.model_state_req.model_state.reference_frame = 'world'

    def reload(self):
        # rospy.wait_for_service('/gazebo/pause_physics')
        # self.pause_proxy()

        rospy.wait_for_service('/gazebo/set_model_state')
        self.model_state_proxy(self.model_state_req)

        rospy.wait_for_service('/gazebo/set_model_configuration')
        self.model_config_proxy(self.model_config_req)

        # rospy.wait_for_service('/gazebo/unpause_physics')
        # self.unpause_proxy()

        rospy.sleep(1)



# class Reloader:
#     def __init__(self, name, pose):
#         self.name = name
#
#         self.state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
#         self.config_service = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
#
#         self.effort_pub = EffortPublisher(name)
#
#         # initial coordinates
#         self.coordinates = Pose()
#         self.coordinates.position.x = pose[0]
#         self.coordinates.position.y = pose[1]
#         self.coordinates.position.z = pose[2]
#         self.coordinates.orientation.x = 0
#         self.coordinates.orientation.y = 0
#         self.coordinates.orientation.z = 0
#         self.coordinates.orientation.w = 0
#         self.position = ModelState()
#         self.position.model_name = self.name
#         self.position.pose = self.coordinates
#
#         # initial joint angles
#         self.joint_state = SetModelConfigurationRequest()
#         self.joint_state.model_name = name
#         self.joint_state.urdf_param_name = 'robot_description'
#         self.joint_state.joint_names = ['left_knee_joint', 'left_knee_joint', 'right_hip_joint', 'right_knee_joint']
#         self.joint_state.joint_positions = np.array([0., 0., 0., 0.])
#
#         # self.joint_publisher = rospy.Publisher(f"/{name}/joint_states", JointState, queue_size=10)
#         # self.joint_pose = JointState()
#         # self.joint_pose.name = ['left_hip_joint', 'right_hip_joint', 'left_knee_joint', 'right_knee_joint']
#         # self.joint_pose.position = [0., 0., 0., 0.]
#
#         self.switch_service = rospy.ServiceProxy('/effort_controller_spawner/switch_controller', SwitchController)
#         self.controllers = [f"/{name}/left_hip_joint_effort_controller/",
#                             f"/{name}/left_knee_joint_effort_controller/",
#                             f"/{name}/right_hip_joint_effort_controller/",
#                             f"/{name}/right_knee_joint_effort_controller/"]
#
#     def restart_controllers(self):
#         rospy.wait_for_service('/effort_controller_spawner/switch_controller')
#         self.switch_service(stop_controllers=self.controllers, start_controllers=[], strictness=2)
#
#         rospy.wait_for_service('/effort_controller_spawner/switch_controller')
#         self.switch_service(stop_controllers=[], start_controllers=self.controllers, strictness=2)
#
#     def reload(self):
#         self.restart_controllers()
#
#         rospy.wait_for_service('/gazebo/set_model_state')
#         response = self.state_service(self.position)
#
#         rospy.wait_for_service('/gazebo/set_model_configuration')
#         response = self.config_service(self.joint_state)
#
#         # self.joint_pose.header.stamp = rospy.Time.now()
#         # self.joint_publisher.publish(self.joint_pose)
#
#         rospy.wait_for_service('/gazebo/set_model_state')
#         # rospy.wait_for_service(f"/{self.name}/joint_states")
#         rospy.wait_for_service('/gazebo/set_model_configuration')
#         # rospy.sleep(3)


def reset_simulation():
    rospy.sleep(1)
    rospy.wait_for_service('/gazebo/pause_physics')
    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.wait_for_service('/gazebo/reset_simulation')
    rospy.wait_for_service('/gazebo/reset_world')
    pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    pause_proxy()
    reset_sim()
    reset_world()
    unpause_proxy()
