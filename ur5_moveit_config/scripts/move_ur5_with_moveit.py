#!/usr/bin/env python
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import sys

def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    return True

def move_ur5():
    # Initialize the moveit_commander and rospy node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5_with_moveit', anonymous=True)

    # Instantiate a RobotCommander object. This object is an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()

    # Instantiate a PlanningSceneInterface object. This object is an interface to the world surrounding the robot.
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints.
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # We can get the name of the reference frame for this robot
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # We can get a list of all the groups in the robot
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the robot
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")

    # Set the target pose for the end effector
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    move_group.set_pose_target(pose_goal)

    # Plan and execute the motion
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()

    # Check if the planning was successful
    current_pose = move_group.get_current_pose().pose
    if all_close(pose_goal, current_pose, 0.01):
        print("Reached the target pose")

if __name__ == '__main__':
    try:
        move_ur5()
    except rospy.ROSInterruptException:
        pass