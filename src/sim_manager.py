#!/usr/bin/env python
import rospy
from ready_signal.srv import Signal, SignalResponse
from std_srvs.srv import Empty


def reset_sim():
    rospy.wait_for_service('/gazebo/pause_physics')
    rospy.wait_for_service('/gazebo/unpause_physics')
    rospy.wait_for_service('/gazebo/reset_simulation')
    rospy.wait_for_service('/gazebo/reset_world')
    pause_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
    reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
    pause_proxy()
    reset_simulation()
    reset_world()
    unpause_proxy()


class SimulationManager:
    def __init__(self, total_nodes):
        self.total_nodes = total_nodes
        self.ready_count = 0
        self.all_signals_received = False

    def handle_ready_signal(self, req):
        self.ready_count += 1
        rospy.loginfo(f"Received ready signal from node. Total ready: {self.ready_count}")

        if self.ready_count >= self.total_nodes:
            reset_sim()
            self.all_signals_received = True
            self.ready_count = 0

        while not self.all_signals_received:
            rospy.sleep(0.01)
        return SignalResponse(all_signals_received=True)


if __name__ == "__main__":
    rospy.init_node('signal_service_node')
    total_nodes = rospy.get_param('~total_nodes')
    service = SimulationManager(total_nodes)
    s = rospy.Service('signal_service', Signal, service.handle_ready_signal)
    rospy.spin()
