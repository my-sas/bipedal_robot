from joint_listener import JointListener
from link_listener import LinkListener

joint_listener = JointListener()
link_listener = LinkListener()

while True:
    joint_data = joint_listener.get_joint_states()
    link_data = link_listener.get_link_states('robot::dummy')
    print(joint_data)
    print(link_data)