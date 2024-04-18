#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetLinkState
from std_msgs.msg import Float64


class LinkListener:
    def __init__(self):
        rospy.init_node('joint_state_listener', anonymous=True)
        rospy.wait_for_service('/gazebo/get_link_state')
        self.service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

    def get_link_states(self, link_name, reference_frame='world'):
        response = self.service(link_name, reference_frame)
        if response.success:
            return response.link_state.pose
        else:
            return None


class JointListener:
    def __init__(self):
        rospy.init_node('joint_state_listener', anonymous=True)
        rospy.Subscriber("/joint_states", JointState, self.callback)
        self.joint_states = None

    def callback(self, data):
        # rospy.loginfo("Data: %s", data)
        self.joint_states = data

    def get_joint_states(self):
        return self.joint_states


class EffortPublisher:
    def __init__(self):
        rospy.init_node('effort_publisher', anonymous=True)

        self.joints = []
        joints = [f'/bipedal_robot/{name}_effort_controller/command' for name in self.joints]

        self.pub = []
        for joint in joints:
            self.pub.append(rospy.Publisher(joint, Float64, queue_size=10))

    def publish(self, efforts: list):
        for i, effort in enumerate(efforts):
            self.pub[i].publish(effort)