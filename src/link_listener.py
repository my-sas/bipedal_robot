#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import GetLinkState


class LinkListener():
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
