#!/usr/bin/env python

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
            y_coord = random.uniform(-0.2, 0.2)
            model_type, model_xml = random.choice([("cube", blue_cube_xml), ("cylinder", red_cylinder_xml)])
            model_name = f"{model_type}_{idx}"
            model_pose = Pose(Point(-0.9, y_coord, 0.95), Quaternion(0, 0, 0, 0))
            spawn_model_prox(model_name, model_xml, "", model_pose, "world")
            idx += 1
            rospy.loginfo(f"Spawned a new {model_type} at (-0.9, {y_coord:.2f}, 0.95)")
            time.sleep(5)
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        spawn_object()
    except rospy.ROSInterruptException:
        pass