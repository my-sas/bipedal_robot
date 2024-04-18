from ros_bridge import JointListener, LinkListener

class Enironment:
    def __init__(self):
        ...
    def step(self, action):
        return self.state, self.reward, done