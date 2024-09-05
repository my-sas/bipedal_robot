import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
# import roslib; roslib.load_manifest('joint_state_publisher')

# import subprocess
#
# rospy.set_param('robot_description', "xacro '$(find bipedal_robot)/urdf/gazebo_bipedal_robot.urdf.xacro'")
#
# subprocess.call(['rosrun', 'robot_state_publisher', 'robot_state_publisher'])


PACKAGE_PATH = rospkg.RosPack().get_path('bipedal_robot')  # path to ros package
URDF_PATH = PACKAGE_PATH + "/urdf/full_bipedal_robot.urdf"  # path to urdf models
N_MODELS = 8  # number of models to spawn
BASE_NAME = "robot"  # model names will be robot1, robot2, etc
SPACE = 10.  # space between robots along y-axis
INIT_POSE = [0., 0., 1.]  # coordinates of model base link


def spawn_model(name, pose):
    """Spawns model from URDF_PATH in gazebo simulation

    Attributes:
        name (str): Name by which you can interact with the model
        pose (list): List of xyz coordinates of model base link
    """

    # rospy.init_node('spawn_robot')
    remap_arg = f"/{name}/joint_states:=/joint_states"
    rospy.init_node('spawn_robot', argv=[remap_arg])

    rospy.wait_for_service('gazebo/spawn_urdf_model')

    try:
        service = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
        with open(URDF_PATH, 'r') as file:
            robot_urdf = file.read()

        # define initial pose of the model
        initial_pose = Pose()
        initial_pose.position.x = pose[0]
        initial_pose.position.y = pose[1]
        initial_pose.position.z = pose[2]  # height above ground

        # call the service to spawn the model
        resp = service(name, robot_urdf, "/", initial_pose, "world")
        print("Spawn status:", resp.success, resp.status_message)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def spawn_models(base_name, n_models, init_pose, space):
    pose = init_pose.copy()
    for i in range(n_models):
        spawn_model(base_name + str(i), pose)
        pose[1] += space


if __name__ == "__main__":
    spawn_models(BASE_NAME, N_MODELS, INIT_POSE, SPACE)







# #!/usr/bin/env python
#
# import rospy
# from gazebo_msgs.srv import SpawnModel
# from geometry_msgs.msg import Pose
# import random
#
#
# def spawn_model(model_name, model_xml, pose):
#     rospy.wait_for_service('/gazebo/spawn_sdf_model')
#     try:
#         spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
#         resp = spawn_urdf(model_name, model_xml, "", pose, "world")
#         return resp.success
#     except rospy.ServiceException as e:
#         rospy.logerr("Service call failed: %s" % e)
#         return False
#
#
# if __name__ == '__main__':
#     rospy.init_node('spawn_models')
#
#     # Загрузите URDF модель
#     with open('./urdf/gazebo_bipedal_robot.urdf.xacro', 'r') as f:
#         model_xml = f.read()
#
#     # Количество моделей для спавна
#     num_models = 10
#
#     for i in range(num_models):
#         model_name = f"model_{i}"
#
#         # Создайте случайную позу
#         pose = Pose()
#         pose.position.x = random.uniform(-5, 5)
#         pose.position.y = random.uniform(-5, 5)
#         pose.position.z = 0.5
#
#         if spawn_model(model_name, model_xml, pose):
#             rospy.loginfo(f"Successfully spawned {model_name}")
#         else:
#             rospy.logerr(f"Failed to spawn {model_name}")
#
#     rospy.loginfo("Finished spawning models")