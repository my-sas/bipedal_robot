import pandas as pd
import rospy
from src.ros_bridge import LinkListener, VelocityListener, ContactListener, Spawner

rospy.init_node("tracker")

rate = rospy.Rate(40)
max_step = 300
step = 0

coordinates_df = pd.DataFrame(columns=["x", "y", "z"])
orientation_df = pd.DataFrame(columns=["x", "y", "z", "w"])
velocity_df = pd.DataFrame(columns=["x", "y", "z"])
left_contact_df = pd.DataFrame(columns=["x_force", "y_force", "z_force", "x_torque", "y_torque", "z_torque"])
right_contact_df = pd.DataFrame(columns=["x_force", "y_force", "z_force", "x_torque", "y_torque", "z_torque"])

spawner = Spawner("robot")
spawner.spawn([0., 0., 10.])

link_listener = LinkListener("robot")
velocity_listener = VelocityListener("robot")
left_foot_contact_listener = ContactListener("robot", "left_foot")
right_foot_contact_listener = ContactListener("robot", "right_foot")

while step < max_step:
    rate.sleep()

    orientation, coordinates = link_listener.get_data("dummy")
    velocity = velocity_listener.get_data()
    left_contact_data = left_foot_contact_listener.get_data()
    right_contact_data = right_foot_contact_listener.get_data()

    coordinates_df.loc[len(coordinates_df)] = coordinates
    orientation_df.loc[len(orientation_df)] = orientation
    velocity_df.loc[len(velocity_df)] = velocity
    left_contact_df.loc[len(left_contact_df)] = left_contact_data
    right_contact_df.loc[len(right_contact_df)] = right_contact_data

    step += 1

coordinates_df.to_csv("logs/coordinates.csv")
orientation_df.to_csv("logs/orientation.csv")
velocity_df.to_csv("logs/velocity.csv")
left_contact_df.to_csv("logs/left_contact.csv")
right_contact_df.to_csv("logs/right_contact.csv")
