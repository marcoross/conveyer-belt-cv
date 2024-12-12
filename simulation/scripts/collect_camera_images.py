#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from pathlib import Path

def image_callback(msg):
    images_path = Path(__file__).parent.parent / 'images'
    bridge = CvBridge()
    try:
        # Convert the ROS Image message to OpenCV2
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    else:
        # Save the image
        n_images_in_folder = len(list(images_path.glob('*.png')))
        image_name = f"camera_image_{n_images_in_folder}.png"
        cv2.imwrite(image_name, cv_image)
        rospy.loginfo("Image saved!")

def main():
    rospy.init_node('camera_subscriber', anonymous=True)
    rospy.Subscriber("/camera_model/camera_sensor/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()