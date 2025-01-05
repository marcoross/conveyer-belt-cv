#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, GripperCommandAction, GripperCommandGoal
from trajectory_msgs.msg import JointTrajectoryPoint
import time

def move_ur5():
    # Check if the ROS node is already initialized
    if not rospy.core.is_initialized():
        print("will be initialized")
        rospy.init_node('move_ur5', anonymous=True)
        print("has been initialized ")

    # Add a delay to ensure the action server is ready
    time.sleep(5)

    # Create an action client for the arm
    arm_client = actionlib.SimpleActionClient('/trajectory_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print("created arm client")
    arm_client.wait_for_server()
    print("arm server is ready")

    # Create an action client for the gripper
    gripper_client = actionlib.SimpleActionClient('/gripper_controller/gripper_cmd', GripperCommandAction)
    print("created gripper client")
    gripper_client.wait_for_server()
    print("gripper server is ready")

    # Create a trajectory goal for the arm
    arm_goal = FollowJointTrajectoryGoal()
    arm_goal.trajectory.joint_names = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]

    # Create a trajectory point for the arm
    arm_point = JointTrajectoryPoint()
    arm_point.positions = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]  # Example positions
    arm_point.time_from_start = rospy.Duration(5.0)  # 5 seconds to reach the position

    # Add the point to the arm goal
    arm_goal.trajectory.points.append(arm_point)

    # Send the arm goal to the action server
    arm_client.send_goal(arm_goal)
    print("sent arm goal")

    # Create a goal for the gripper
    gripper_goal = GripperCommandGoal()
    gripper_goal.command.position = 0.8  # Example position (open)
    gripper_goal.command.max_effort = 10.0  # Example effort

    # Send the gripper goal to the action server
    gripper_client.send_goal(gripper_goal)
    print("sent gripper goal")

    # Wait for the results
    arm_client.wait_for_result()
    gripper_client.wait_for_result()

if __name__ == '__main__':
    try:
        move_ur5()
    except rospy.ROSInterruptException:
        pass