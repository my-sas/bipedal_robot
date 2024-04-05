#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetLinkState


def get_link_coordinates(link_name, reference_frame="world"):
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        link_state_response = get_link_state(link_name, reference_frame)

        if link_state_response.success:
            position = link_state_response.link_state.pose.position
            orientation = link_state_response.link_state.pose.orientation
            return (position, orientation)
        else:
            rospy.logerr("Failed to get link state: %s", link_state_response.status_message)
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return None


if __name__ == "__main__":
    rospy.init_node('get_link_coordinates_node')

    link_name = 'robot::dummy'  # Replace with your link name as defined in Gazebo
    while not rospy.is_shutdown():
        result = get_link_coordinates(link_name)
        if result:
            position, orientation = result
            rospy.loginfo("Link [%s] Position: [%f, %f, %f]", link_name, position.x, position.y, position.z)
            rospy.loginfo("Link [%s] Orientation: [%f, %f, %f, %f]", link_name, orientation.x, orientation.y,
                          orientation.z, orientation.w)
        else:
            rospy.logerr("No link state received for link [%s]", link_name)

        rospy.sleep(1)  # Sleep for a second before trying again