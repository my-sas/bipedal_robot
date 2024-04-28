#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelConfiguration, SetModelConfigurationRequest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from controller_manager_msgs.srv import SwitchController
rospy.init_node('ros_bridge', anonymous=True)


class LinkListener:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_link_state')
        self.service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
    def get_data(self, link_name, reference_frame='world'):
        rospy.wait_for_service('/gazebo/get_link_state')
        response = self.service(link_name, reference_frame)
        if response.success:
            link_states = response.link_state.pose.position
            return [link_states.x, link_states.y, link_states.z]
        else:
            return None


class JointListener:
    def __init__(self):
        rospy.Subscriber("/joint_states", JointState, self.callback, queue_size=30)
        self.joint_states = None

    def callback(self, data):
        # rospy.loginfo("Data: %s", data)
        self.joint_states = list(data.position) + list(data.velocity)

    def get_data(self):
        return self.joint_states


class EffortPublisher:
    def __init__(self):
        joints = ['left_knee_joint', 'left_knee_joint', 'right_hip_joint', 'right_knee_joint']
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


class Resetter:
    def __init__(self):
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

    def reset(self):
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
    rospy.sleep(1)
