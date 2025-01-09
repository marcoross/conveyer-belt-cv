#!/usr/bin/env python
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander
import sys

def open_gripper(gripper_group):
    gripper_group.set_named_target("open")
    plan = gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()

def close_gripper(gripper_group):
    gripper_group.set_named_target("closed")
    plan = gripper_group.go(wait=True)
    gripper_group.stop()
    gripper_group.clear_pose_targets()

def test_gripper():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('test_gripper_moveit', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    gripper_group = MoveGroupCommander("gripper")

    rospy.sleep(2)  # Allow time for the gripper to initialize

    open_gripper(gripper_group)
    rospy.sleep(2)  # Wait for the gripper to open

    close_gripper(gripper_group)
    rospy.sleep(2)  # Wait for the gripper to close

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        test_gripper()
    except rospy.ROSInterruptException:
        pass