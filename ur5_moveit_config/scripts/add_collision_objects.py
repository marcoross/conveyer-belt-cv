#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import sys

def add_collision_objects():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('add_collision_objects', anonymous=True)

    scene = PlanningSceneInterface()
    rospy.sleep(2)  # Allow time for the planning scene to initialize

    # Robot's actual position in the world frame, because we need the other positions to be relative to the robot
    robot_position_x = 0.65  # Example position
    robot_position_y = 0.0  # Example position
    robot_position_z = 1  # Example position

    # Define the conveyor belt
    conveyor_pose = PoseStamped()
    conveyor_pose.header.frame_id = "world"
    conveyor_pose.pose.position.x =  0.0 - robot_position_x# Adjusted position
    conveyor_pose.pose.position.y = 0.0 - robot_position_y # Adjusted position
    conveyor_pose.pose.position.z = 0.85 - robot_position_z  # Adjusted position
    conveyor_pose.pose.orientation.w = 1.0

    scene.add_box("conveyor_belt", conveyor_pose, size=(2.0, 0.5, 0.1))

    # Define the camera
    camera_pose = PoseStamped()
    camera_pose.header.frame_id = "world"
    camera_pose.pose.position.x =  0.0 - robot_position_x  # Adjusted position
    camera_pose.pose.position.y = 0.0  - robot_position_y  # Adjusted position
    camera_pose.pose.position.z =  1.55 - robot_position_z  # Adjusted position
    camera_pose.pose.orientation.w = 1.0

    scene.add_box("camera", camera_pose, size=(0.1, 0.1, 0.1))

    rospy.sleep(2)  # Allow time for the objects to be added to the planning scene

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        add_collision_objects()
    except rospy.ROSInterruptException:
        pass