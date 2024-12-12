#!/usr/bin/python

import rospy
from geometry_msgs.msg import Twist
# import logging

def conveyor_belt_controller():
    rospy.init_node('conveyor_belt_controller', anonymous=True)
    pub = rospy.Publisher('/conveyor_belt/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10 Hz

    while not rospy.is_shutdown():
        twist = Twist()
        twist.linear.x = 0.05 # Set the conveyor belt speed
        pub.publish(twist)
        # logging.info("Conveyor belt speed: %s", twist.linear.x)
        rate.sleep()

if __name__ == '__main__':
    try:
        conveyor_belt_controller()
    except rospy.ROSInterruptException:
        pass