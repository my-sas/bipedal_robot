#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

sas = None

# This callback function will be called every time a new message is published on the /joint_states topic.
def joint_states_callback(msg):
    # Process the joint states data (positions, velocities, efforts)
    positions = msg.position
    velocities = msg.velocity
    efforts = msg.effort
    sas = positions
    rospy.loginfo('Done')


if __name__ == '__main__':
    rospy.init_node('joint_state_listener', anonymous=True)
    rospy.Subscriber("/bipedal_robot/joint_states", JointState, joint_states_callback)
    rospy.loginfo(f'data: {sas}')

    # Keep the program alive.
    rospy.spin()