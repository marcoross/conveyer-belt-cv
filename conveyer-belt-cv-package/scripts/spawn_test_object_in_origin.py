#!/usr/bin/env python
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import time

def spawn_single_object():
    rospy.init_node('spawn_single_object')

    # Get the path to the models
    rospack = rospkg.RosPack()
    models_path = rospack.get_path('conveyer-belt-cv-package') + '/models'

    # Load the model XML
    with open(models_path + "/small_blue_cube/model.sdf", "r") as f:
        red_cylinder_xml = f.read()

    # Define the position and orientation of the object
    model_pose = Pose(Point(0, 0, 1), Quaternion(0, 0, 0, 1))

    # Wait for the spawn service to be available
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    current_time = time.strftime("%Y%m%d%H%M%S")

    # Create a unique name for the object
    object_name = "single_red_cylinder_" + current_time

    

    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(object_name, red_cylinder_xml, "", model_pose, "world")
        rospy.loginfo("Spawned a single red cylinder at (0, 0, 1)")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        spawn_single_object()
    except rospy.ROSInterruptException:
        pass