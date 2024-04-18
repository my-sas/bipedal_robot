#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

class JointListener():
    def __init__(self):
        rospy.init_node('joint_state_listener', anonymous=True)
        rospy.Subscriber("/joint_states", JointState, self.callback)
        self.joint_states = None

    def callback(self, data):
        # rospy.loginfo("Data: %s", data)
        self.joint_states = data

    def get_joint_states(self):
        return self.joint_states