#!/usr/bin/env python
import rospy
import moveit_commander
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import math
import sys
from control_msgs.msg import GripperCommandGoal
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal


# Initialize the moveit_commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_ur5_with_moveit', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
move_group = moveit_commander.MoveGroupCommander("manipulator")

print("============ Available Planning Groups:", robot.get_group_names())

# Sometimes for debugging it is useful to print the entire state of the robot
print("============ Printing robot state")
print(robot.get_current_state())
print("")



# Create a goal for the gripper
gripper_client = actionlib.SimpleActionClient('/gripper_controller/gripper_cmd', GripperCommandAction)
gripper_client.wait_for_server()

gripper_goal = GripperCommandGoal()
gripper_goal.command.position = 0.0  # Position to pick up items with a diameter of 5 cm
gripper_goal.command.max_effort = 1.0  # Example effort

# Send the gripper goal to the action server
gripper_client.send_goal(gripper_goal)
print("sent gripper goal")

# Wait for the result
gripper_client.wait_for_result()

robot_position_x = 0.625  # Example position
robot_position_y = 0.0  # Example position


# Set the target pose for the gripper's base link
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.0 - robot_position_x # Position of the object
pose_goal.position.y = 0.0 - robot_position_y
pose_goal.position.z = 0.15

# Use quaternion_from_euler to set the orientation
# Assuming you want the gripper to be parallel to the ground plane with no rotation around the Z-axis
roll = 0.0
pitch = 1/2 * math.pi
yaw = 0 #math.radians(0)  # Change this value to rotate the gripper around the Z-axis
q = quaternion_from_euler(roll, pitch, yaw)
pose_goal.orientation.x = q[0]
pose_goal.orientation.y = q[1]
pose_goal.orientation.z = q[2]
pose_goal.orientation.w = q[3]


# Set the pose target for the gripper's base link
gripper_base_link = "robotiq_85_base_link"  # Replace with the actual base link of your gripper
move_group.set_pose_target(pose_goal, gripper_base_link)

# Increase the speed of the robot
move_group.set_max_velocity_scaling_factor(1.0)  # Set to 1.0 for maximum speed
move_group.set_max_acceleration_scaling_factor(0.3)  # Set to 1.0 for maximum acceleration


# Plan and execute the motion
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()
print("Stopped the motion")



# Set the target pose for the gripper's base link
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.0 - robot_position_x # Position of the object
pose_goal.position.y = 0.0 - robot_position_y
pose_goal.position.z = 0.07

# Use quaternion_from_euler to set the orientation
# Assuming you want the gripper to be parallel to the ground plane with no rotation around the Z-axis
roll = 0.0
pitch = 1/2 * math.pi
yaw = 0 #math.radians(0)  # Change this value to rotate the gripper around the Z-axis
q = quaternion_from_euler(roll, pitch, yaw)
pose_goal.orientation.x = q[0]
pose_goal.orientation.y = q[1]
pose_goal.orientation.z = q[2]
pose_goal.orientation.w = q[3]


# Set the pose target for the gripper's base link
gripper_base_link = "robotiq_85_base_link"  # Replace with the actual base link of your gripper
move_group.set_pose_target(pose_goal, gripper_base_link)



# Plan and execute the motion
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()
print("Stopped the motion")


# Create a goal for the gripper
gripper_client = actionlib.SimpleActionClient('/gripper_controller/gripper_cmd', GripperCommandAction)
gripper_client.wait_for_server()

gripper_goal = GripperCommandGoal()
gripper_goal.command.position = 0.39  # Position to pick up items with a diameter of 5 cm
gripper_goal.command.max_effort = 0.4  # Example effort

# Send the gripper goal to the action server
gripper_client.send_goal(gripper_goal)
print("sent gripper goal")

# Wait for the result
gripper_client.wait_for_result()

gripper_client.wait_for_server()

gripper_goal = GripperCommandGoal()
gripper_goal.command.position = 0.37175  # Position to pick up items with a diameter of 5 cm
gripper_goal.command.max_effort = 1.0  # Example effort

# Send the gripper goal to the action server
gripper_client.send_goal(gripper_goal)
print("sent gripper goal")

# Wait for the result
gripper_client.wait_for_result()




# Set the target pose for the gripper's base link
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 0.0 - robot_position_x # Position of the object
pose_goal.position.y = 0.0 - robot_position_y
pose_goal.position.z = 0.15

# Use quaternion_from_euler to set the orientation
# Assuming you want the gripper to be parallel to the ground plane with no rotation around the Z-axis
roll = 0.0
pitch = 1/2 * math.pi
yaw = 0 #math.radians(0)  # Change this value to rotate the gripper around the Z-axis
q = quaternion_from_euler(roll, pitch, yaw)
pose_goal.orientation.x = q[0]
pose_goal.orientation.y = q[1]
pose_goal.orientation.z = q[2]
pose_goal.orientation.w = q[3]


# Set the pose target for the gripper's base link
gripper_base_link = "robotiq_85_base_link"  # Replace with the actual base link of your gripper
move_group.set_pose_target(pose_goal, gripper_base_link)

# Increase the speed of the robot


# Plan and execute the motion
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()
print("Stopped the motion")



# Set the target pose for the gripper's base link
pose_goal = geometry_msgs.msg.Pose()
pose_goal.position.x = 1.25 - robot_position_x # Position of the object
pose_goal.position.y = - 0.0 - robot_position_y
pose_goal.position.z = 0.1

# Use quaternion_from_euler to set the orientation
# Assuming you want the gripper to be parallel to the ground plane with no rotation around the Z-axis
roll = 0.0
pitch = 1/2 * math.pi
yaw = 0 #math.radians(0)  # Change this value to rotate the gripper around the Z-axis
q = quaternion_from_euler(roll, pitch, yaw)
pose_goal.orientation.x = q[0]
pose_goal.orientation.y = q[1]
pose_goal.orientation.z = q[2]
pose_goal.orientation.w = q[3]



# Set the pose target for the gripper's base link
gripper_base_link = "robotiq_85_base_link"  # Replace with the actual base link of your gripper
move_group.set_pose_target(pose_goal, gripper_base_link)

# Increase the speed of the robot


# Plan and execute the motion
plan = move_group.go(wait=True)
move_group.stop()
move_group.clear_pose_targets()
print("Stopped the motion")


# Create a goal for the gripper
gripper_client = actionlib.SimpleActionClient('/gripper_controller/gripper_cmd', GripperCommandAction)
gripper_client.wait_for_server()

gripper_goal = GripperCommandGoal()
gripper_goal.command.position = 0.0  # Position to pick up items with a diameter of 5 cm
gripper_goal.command.max_effort = 1.0  # Example effort

# Send the gripper goal to the action server
gripper_client.send_goal(gripper_goal)
print("sent gripper goal")

# Wait for the result
gripper_client.wait_for_result()



moveit_commander.roscpp_shutdown()