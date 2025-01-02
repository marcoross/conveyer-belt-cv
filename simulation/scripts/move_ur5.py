#!/usr/bin/env python
import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

def move_ur5():
    # Initialize the ROS node
    rospy.init_node('move_ur5', anonymous=True)

    # Create an action client
    client = actionlib.SimpleActionClient('/eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    # Create a trajectory goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]

    # Create a trajectory point
    point = JointTrajectoryPoint()
    point.positions = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]  # Straight-up position
    point.time_from_start = rospy.Duration(5.0)  # 5 seconds to reach the position

    # Add the point to the goal
    goal.trajectory.points.append(point)

    # Send the goal to the action server
    client.send_goal(goal)
    client.wait_for_result()

if __name__ == '__main__':
    try:
        move_ur5()
    except rospy.ROSInterruptException:
        pass