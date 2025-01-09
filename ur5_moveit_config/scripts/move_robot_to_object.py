#!/usr/bin/env python
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import math
from geometry_msgs.msg import Point
from control_msgs.msg import GripperCommandGoal, GripperCommandAction
import actionlib
import sys

class MoveRobotToObject:
    def __init__(self):
        rospy.init_node('move_robot_to_object', anonymous=True)

        # Initialize MoveIt commander
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.move_group = moveit_commander.MoveGroupCommander("manipulator")
        self.gripper_client = actionlib.SimpleActionClient('/gripper_controller/gripper_action', GripperCommandAction)
        self.gripper_client.wait_for_server()

        # Subscribe to the object coordinates topic
        rospy.Subscriber('/object_coordinates', Point, self.object_coordinates_callback)

    def object_coordinates_callback(self, msg):
        # Extract coordinates from the message
        x = msg.x
        y = msg.y
        object_type = msg.z  # Assuming object type is encoded in the z-coordinate

        # Move the robot to the specified coordinates
        self.move_to_coordinates(x, y)

        # Close the gripper to pick up the object
        self.close_gripper()

    def move_to_coordinates(self, x, y):
        # Set the target pose for the end effector
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = 0.15  # Adjust the height as needed

        # Set the orientation to point downward
        roll = 0.0
        pitch = math.pi / 2  # 90 degrees
        yaw = 0.0
        q = quaternion_from_euler(roll, pitch, yaw)
        pose_goal.orientation.x = q[0]
        pose_goal.orientation.y = q[1]
        pose_goal.orientation.z = q[2]
        pose_goal.orientation.w = q[3]

        # Set the pose target for the end effector link
        end_effector_link = self.move_group.get_end_effector_link()
        self.move_group.set_pose_target(pose_goal, end_effector_link)

        # Plan and execute the motion
        plan = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        rospy.loginfo("Moved to coordinates ({}, {})".format(x, y))

    def close_gripper(self):
        # Create a goal for the gripper
        gripper_goal = GripperCommandGoal()
        gripper_goal.command.position = 0.02  # Adjust the position to partially close the gripper
        gripper_goal.command.max_effort = 1.0  # Example effort

        # Send the gripper goal to the action server
        self.gripper_client.send_goal(gripper_goal)
        self.gripper_client.wait_for_result()
        rospy.loginfo("Closed the gripper to pick up the object")

if __name__ == '__main__':
    try:
        move_robot_to_object = MoveRobotToObject()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass