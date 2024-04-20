from ros_bridge import JointListener, LinkListener, EffortPublisher, VelocityListener
# from reward import Reward


class Environment:
    def __init__(self):
        self.joint_listener = JointListener()
        self.link_listener = LinkListener()
        self.effort_publisher = EffortPublisher()
        self.velocity_listener = VelocityListener()

    # def is_done(self, link_data):
    #     ...
    # def reset(self):
    #     ...

    def step(self, action: list) -> (list, int, bool):
        self.effort_publisher.send(action)
        joint_data = self.joint_listener.get_data()
        link_data = self.link_listener.get_data('robot::dummy')
        velocity_data = self.velocity_listener.get_data()
        state = (velocity_data)
        reward = 0.0 # Reward(link_data)
        done = False # self.is_done(link_data)
        return state, reward, done
