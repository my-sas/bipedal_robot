#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty
from std_msgs.msg import Float64
rospy.init_node('ros_bridge', anonymous=True)


class LinkListener:
    def __init__(self):
        rospy.wait_for_service('/gazebo/get_link_state')
        self.service = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)

    def get_data(self, link_name, reference_frame='world'):
        response = self.service(link_name, reference_frame)
        if response.success:
            link_states = response.link_state.pose.position
            return [link_states.x, link_states.y, link_states.z]
        else:
            return None


class JointListener:
    def __init__(self):
        rospy.Subscriber("/joint_states", JointState, self.callback)
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


def reset_simulation():
    rospy.wait_for_service('/gazebo/reset_simulation')
    reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_sim()
