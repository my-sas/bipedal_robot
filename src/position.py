#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetLinkState


class RobotStateMonitor:
    def __init__(self):
        rospy.init_node('robot_state_monitor')

        self.joint_state_subscriber = rospy.Subscriber(
            "/joint_states", JointState, self.joint_state_callback)
        self.joint_states = None

        rospy.wait_for_service('/gazebo/get_link_state')

        self.get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

        self.links = ['dummy']
        self.robot = 'robot'

    def joint_state_callback(self, msg):
        self.joint_states = msg

    def get_link_coordinates(self):
        response = []
        for link in self.links:
            link_name = f'{self.robot}::{link}'
            try:
                link_state_response = self.get_link_state(link_name, 'world')
                position = link_state_response.link_state.pose.position
                orientation = link_state_response.link_state.pose.orientation
                rospy.loginfo(
                    "Link [%s] Position: [%f, %f, %f]",
                    link, position.x, position.y, position.z)
                rospy.loginfo(
                    "Link [%s] Orientation: [%f, %f, %f, %f]",
                    link, orientation.x, orientation.y, orientation.z, orientation.w)
                response.append((position, orientation))
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s", e)

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.joint_states is not None:
                rospy.loginfo("Joint positions: %s", self.joint_states.position)
            self.get_link_coordinates()
            rate.sleep()


if __name__ == "__main__":
    monitor = RobotStateMonitor()
    monitor.run()
