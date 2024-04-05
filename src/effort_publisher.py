#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64


def move_joint():
    # Get joints names
    joints = ['left_hip_joint',
              'left_knee_joint',
              'right_hip_joint',
              'right_knee_joint']
    joints = [f'/bipedal_robot/{name}_effort_controller/command' for name in joints]

    # Initialize the ROS Node
    rospy.init_node('effort_controller', anonymous=True)

    # Create a publisher object
    pub = []
    for joint in joints:
        pub.append(rospy.Publisher(joint, Float64, queue_size=10))

    # Set the loop rate (in Hz)
    rate = rospy.Rate(50) # 10 Hz

    # Initialize effort value
    effort = 11.0 # Change this value to whatever is appropriate

    # Keep publishing until a Ctrl-C is pressed
    while not rospy.is_shutdown():
        # rospy.loginfo("Publishing effort: %f" % effort)
        pub[0].publish(effort)
    rate.sleep()


if __name__ == '__main__':
    try:
        move_joint()
    except rospy.ROSInterruptException:
        pass
