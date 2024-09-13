"""
This module provides interface for interaction with models in gazebo simulation.
You can send model to move joint, get model coordinates, spawn models in simulation, etc.
"""

import time
import numpy as np
import subprocess

import rospy
import rospkg
# from gazebo_msgs.srv import SpawnModel
# from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetLinkState
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
# from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Float64

from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Empty

# from geometry_msgs.msg import Pose
# from std_srvs.srv import Empty
# from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
# from gazebo_msgs.msg import ModelState
# from gazebo_msgs.srv import GetModelState, GetModelStateRequest
# from controller_manager_msgs.srv import SwitchController

# rospy.init_node("ros_bridge", anonymous=True)


PACKAGE_PATH = rospkg.RosPack().get_path("bipedal_robot")  # path to ros package
URDF_PATH = PACKAGE_PATH + "/urdf/full_bipedal_robot.urdf"  # path to urdf models
N_MODELS = 8  # number of models to spawn
BASE_NAME = "robot"  # model names will be robot1, robot2, etc
SPACE = 10.  # space between robots along y-axis
INIT_POSE = [0., 0., 1.]  # coordinates of model base link


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
                [orientation.x, orientation.y, orientation.z, orientation.w],
                [coordinates.x, coordinates.y, coordinates.z])
        else:
            return None


class JointListener:
    def __init__(self, name):
        rospy.Subscriber(f"/{name}/joint_states", JointState, self.callback, queue_size=100)
        self.joint_states = None

    def callback(self, data):
        # rospy.loginfo("Data: %s", data)
        self.joint_states = list(data.position) + list(data.velocity)

    def get_data(self):
        return self.joint_states


class EffortPublisher:
    def __init__(self, name):
        self.name = name
        joints = ['left_hip_joint', 'right_hip_joint', 'left_knee_joint', 'right_knee_joint']
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
        self.velocity = data.twist[robot_index].linear.x

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



# cant do this class yet
# class ContactListener:
#     def __init__(self):
#         rospy.Subscriber("/gazebo/base_collision", ContactsState, self.callback)
#         self.contacts = None
#
#     def callback(self, data):
#         self.contacts = data
#         # self.contacts = [[force for force in contact]  for contact in data.states]
#
#     def get_data(self):
#         return self.contacts


# class Spawner:
#     def __init__(self, base_name, n_models, init_pose, space):
#         rospy.wait_for_service('gazebo/spawn_urdf_model')
#         self.service = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
#         self.base_name = base_name
#         self.n_models = n_models
#         self.init_pose = init_pose
#         self.space = space
#
#         self.names = [base_name + str(i) for i in range(n_models)]
#         self.poses = []
#         for i in range(n_models):
#             pose = init_pose.copy()
#             pose[1] += space
#             self.poses.append(pose)
#
#     def spawn_model(self, name, pose):
#         spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
#         # open urdf file
#         with open(URDF_PATH, 'r') as file:
#             robot_urdf = file.read()
#
#         # define initial pose of the model
#         initial_pose = Pose()
#         initial_pose.position.x = pose[0]
#         initial_pose.position.y = pose[1]
#         initial_pose.position.z = pose[2]  # height above ground
#
#         # call the service to spawn the model
#         response = self.service(name, robot_urdf, "", initial_pose, "world")
#         print("Spawn status:", response.success, response.status_message)
#
#     def spawn_models(self):
#         for name, pose in zip(self.names, self.poses):
#             self.spawn_model(name, pose)


# class Despawner:
#     def __init__(self):
#         rospy.wait_for_service('/gazebo/delete_model')
#         self.service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
#
#     def delete_model(self):
#         response = self.service(model_name)
#         print(response.status_message)


# class Launcher:
#     def __init__(self):



class Reloader:
    def __init__(self, name, pose):
        self.name = name

        self.state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.config_service = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        self.effort_pub = EffortPublisher(name)

        # initial coordinates
        self.coordinates = Pose()
        self.coordinates.position.x = pose[0]
        self.coordinates.position.y = pose[1]
        self.coordinates.position.z = pose[2]
        self.coordinates.orientation.x = 0
        self.coordinates.orientation.y = 0
        self.coordinates.orientation.z = 0
        self.coordinates.orientation.w = 0
        self.position = ModelState()
        self.position.model_name = self.name
        self.position.pose = self.coordinates

        # initial joint angles
        self.joint_state = SetModelConfigurationRequest()
        self.joint_state.model_name = name
        self.joint_state.urdf_param_name = 'robot_description'
        self.joint_state.joint_names = ['left_knee_joint', 'left_knee_joint', 'right_hip_joint', 'right_knee_joint']
        self.joint_state.joint_positions = np.array([0., 0., 0., 0.])

        # self.joint_publisher = rospy.Publisher(f"/{name}/joint_states", JointState, queue_size=10)
        # self.joint_pose = JointState()
        # self.joint_pose.name = ['left_hip_joint', 'right_hip_joint', 'left_knee_joint', 'right_knee_joint']
        # self.joint_pose.position = [0., 0., 0., 0.]

        self.switch_service = rospy.ServiceProxy('/effort_controller_spawner/switch_controller', SwitchController)
        self.controllers = [f"/{name}/left_hip_joint_effort_controller/",
                            f"/{name}/left_knee_joint_effort_controller/",
                            f"/{name}/right_hip_joint_effort_controller/",
                            f"/{name}/right_knee_joint_effort_controller/"]

    def restart_controllers(self):
        rospy.wait_for_service('/effort_controller_spawner/switch_controller')
        self.switch_service(stop_controllers=self.controllers, start_controllers=[], strictness=2)

        rospy.wait_for_service('/effort_controller_spawner/switch_controller')
        self.switch_service(stop_controllers=[], start_controllers=self.controllers, strictness=2)

    def reload(self):
        self.restart_controllers()

        rospy.wait_for_service('/gazebo/set_model_state')
        response = self.state_service(self.position)

        rospy.wait_for_service('/gazebo/set_model_configuration')
        response = self.config_service(self.joint_state)

        # self.joint_pose.header.stamp = rospy.Time.now()
        # self.joint_publisher.publish(self.joint_pose)

        rospy.wait_for_service('/gazebo/set_model_state')
        # rospy.wait_for_service(f"/{self.name}/joint_states")
        rospy.wait_for_service('/gazebo/set_model_configuration')
        # rospy.sleep(3)

def wait_for_reset():
    rospy.wait_for_service('node_ready')
    ready_service = rospy.ServiceProxy('node_ready', ReadySignal)
    resp = ready_service()




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
