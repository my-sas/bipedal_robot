"""
This module provides interface for interaction with models in gazebo simulation.
You can send model to move joint, get model coordinates, spawn models in simulation, etc.
"""

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from gazebo_msgs.srv import GetLinkState
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from std_msgs.msg import Float64
# from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
# from gazebo_msgs.msg import ModelState
# from gazebo_msgs.srv import GetModelState, GetModelStateRequest
# from controller_manager_msgs.srv import SwitchController
rospy.init_node("ros_bridge", anonymous=True)


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
        response = self.service(f"{self.name}:{link_name}", reference_frame)
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
        rospy.Subscriber(f"/{name}/joint_states", JointState, self.callback, queue_size=30)
        self.joint_states = None

    def callback(self, data):
        # rospy.loginfo("Data: %s", data)
        self.joint_states = list(data.position) + list(data.velocity)

    def get_data(self):
        return self.joint_states


class EffortPublisher:
    def __init__(self):
        joints = ['left_hip_joint', 'right_hip_joint', 'left_knee_joint', 'right_knee_joint']
        joints = [f'/bipedal_robot/{name}_effort_controller/command' for name in joints]
        
        self.pub = []
        for joint in joints:
            self.pub.append(rospy.Publisher(joint, Float64, queue_size=10))

    def send(self, efforts: list):
        for i, effort in enumerate(efforts):
            self.pub[i].publish(effort)


class VelocityListener:
    def __init__(self):
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.velocity = None

    def callback(self, data):
        robot_index = data.name.index('robot')
        self.velocity = data.twist[robot_index].linear.x

    def get_data(self):
        return self.velocity


class ContactListener:
    def __init__(self):
        rospy.Subscriber("/gazebo/base_collision", ContactsState, self.callback)
        self.contacts = None

    def callback(self, data):
        self.contacts = data
        # self.contacts = [[force for force in contact]  for contact in data.states]

    def get_data(self):
        return self.contacts


class Spawner:
    def __init__(self, base_name, n_models, init_pose, space):
        rospy.wait_for_service('gazebo/spawn_urdf_model')
        self.service = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        self.base_name = base_name
        self.n_models = n_models
        self.init_pose = init_pose
        self.space = space

        self.names = [base_name + str(i) for i in range(n_models)]
        self.poses = []
        for i in range(n_models):
            pose = init_pose.copy()
            pose[1] += space
            self.poses.append(pose)

    def spawn_model(self, name, pose):
        spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        # open urdf file
        with open(URDF_PATH, 'r') as file:
            robot_urdf = file.read()

        # define initial pose of the model
        initial_pose = Pose()
        initial_pose.position.x = pose[0]
        initial_pose.position.y = pose[1]
        initial_pose.position.z = pose[2]  # height above ground

        # call the service to spawn the model
        response = self.service(name, robot_urdf, "", initial_pose, "world")
        print("Spawn status:", response.success, response.status_message)

    def spawn_models(self):
        for name, pose in zip(self.names, self.poses):
            self.spawn_model(name, pose)


class Despawner:
    def __init__(self):
        rospy.wait_for_service('/gazebo/delete_model')
        self.service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

    def delete_model(self):
        response = self.service(model_name)
        print(response.status_message)


class Reloader:
    def __init__(self, name):
        self.pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.model_config_proxy = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        self.starting_pos = np.array([0., 0., 0., 0.])
        self.joint_name_lst = ['left_knee_joint', 'left_knee_joint', 'right_hip_joint', 'right_knee_joint']

        self.model_config_req = SetModelConfigurationRequest()
        self.model_config_req.model_name = 'robot'
        self.model_config_req.urdf_param_name = 'robot_description'
        self.model_config_req.joint_names = self.joint_name_lst
        self.model_config_req.joint_positions = self.starting_pos

        self.model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.model_state_req = SetModelStateRequest()
        self.model_state_req.model_state = ModelState()
        self.model_state_req.model_state.model_name = 'robot'

        self.model_state_req.model_state.pose.position.x = 0.0
        self.model_state_req.model_state.pose.position.y = 0.0
        self.model_state_req.model_state.pose.position.z = 2.5
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

        self.get_model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.get_model_state_req = GetModelStateRequest()
        self.get_model_state_req.model_name = 'robot'
        self.get_model_state_req.relative_entity_name = 'world'

    def switch_controllers(self, stop_controllers, start_controllers):
        rospy.wait_for_service('/controller_manager/switch_controller')
        try:
            switch_service = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
            switch_service(stop_controllers, start_controllers, 2, False, 0.0)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    def reload(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        self.pause_proxy()

        # self.switch_controllers(['effort_controller'], [])

        rospy.wait_for_service('/gazebo/set_model_state')
        self.model_state_proxy(self.model_state_req)

        rospy.wait_for_service('/gazebo/set_model_configuration')
        self.model_config_proxy(self.model_config_req)

        rospy.wait_for_service('/gazebo/unpause_physics')
        self.unpause_proxy()

        rospy.sleep(3)
#
#
#
#
#
#
#
# def reset_simulation():
#     rospy.sleep(1)
#     rospy.wait_for_service('/gazebo/pause_physics')
#     rospy.wait_for_service('/gazebo/unpause_physics')
#     rospy.wait_for_service('/gazebo/reset_simulation')
#     rospy.wait_for_service('/gazebo/reset_world')
#     pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
#     unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
#     reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
#     reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
#     pause_proxy()
#     reset_sim()
#     reset_world()
#     unpause_proxy()
#     rospy.sleep(1)
