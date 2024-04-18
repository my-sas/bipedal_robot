#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def callback(data):
    rospy.loginfo("Data: %s", data)

def listener():
    rospy.init_node('joint_state_listener', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()