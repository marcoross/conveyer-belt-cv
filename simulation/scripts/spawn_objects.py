#!/usr/bin/python

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import time
from pathlib import Path
import random



def spawn_object():
    rospy.init_node('spawn_objects_node', anonymous=True)
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    models_path = Path(__file__).parent.parent / "models"
    
    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        with open(models_path / "small_blue_cube" / "model.sdf", "r") as f:
            blue_cube_xml = f.read()
            
        with open(models_path / "small_red_cylinder" / "model.sdf", "r") as f:
            red_cylinder_xml = f.read()
        
        time.sleep(2)
        idx = 0
        while not rospy.is_shutdown():
            model_name = f"small_blue_cube_{idx}"
            y_coord = random.uniform(-0.2, 0.2)
            model_xml = random.choice([blue_cube_xml, red_cylinder_xml])
            model_pose = Pose(Point(-0.9, y_coord, 1.05), Quaternion(0, 0, 0, 0))
            spawn_model_prox(model_name, model_xml, "", model_pose, "world")
            idx += 1
            rospy.loginfo(f"Spawned a new small_blue_cube at (-0.9, {y_coord:.2f}, 1.05)")
            time.sleep(5)
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        spawn_object()
    except rospy.ROSInterruptException:
        pass