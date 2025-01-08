#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import String
import time


def pick_object(data: Point):
        rospy.loginfo("Received coordinates: x=%f, y=%f, z=%f", data.x, data.y, data.z)
        object_type = "cube" if data.z == 0 else "cylinder"
        drop_y = 0.5 if object_type == "cube" else -0.5
        
        pickup_coords = (data.x, data.y, 0.925)
        drop_coords = (0.75, drop_y, 0.925)      
        rospy.loginfo(f"Moving object from {pickup_coords} to {drop_coords}")          
        
        # TODO Convert pickup_coords and drop_coords to robot coordinate system and add the code to move the robot
        time.sleep(10)
        
        pub = rospy.Publisher("/objects_placed", String, queue_size=10)
        pub.publish(String("done"))
        rospy.loginfo(f"Done picking up {object_type}")


def pick_objects_loop():
    rospy.init_node('controller_node', anonymous=True)
    rospy.Subscriber('/object_coordinates', Point, pick_object)
    pub = rospy.Publisher("/objects_placed", String, queue_size=10)
    pub.publish(String("init"))  # init message to work init the topic
    rospy.spin()
    

if __name__ == '__main__':
    try:
        pick_objects_loop()
    except rospy.ROSInterruptException:
        pass