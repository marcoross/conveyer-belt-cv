#!/usr/bin/env python

import numpy as np
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import String
import time
from pathlib import Path
import random
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from ultralytics import YOLO
import torch
from threading import Thread


spawn_objects = True
captured_image = None


def init_resources(models_path):
    spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
    with open(models_path / "small_blue_cube" / "model.sdf", "r") as f:
        blue_cube_xml = f.read()
            
    with open(models_path / "small_red_cylinder" / "model.sdf", "r") as f:
        red_cylinder_xml = f.read()
    return spawn_model_prox,blue_cube_xml,red_cylinder_xml


def pickup_objects(yolo):
    # Stop the conveyor belt
    conveyor_pub = rospy.Publisher("/conveyor_belt/cmd_vel", Twist, queue_size=10)
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    conveyor_pub.publish(stop_msg)
    rospy.loginfo("Sent message to stop conveyor belt")
    # Stop spawning objects
    global spawn_objects
    spawn_objects = False
    time.sleep(2)
    
    # Perform object detection again, after the belt has stopped
    results = yolo(captured_image)
    obb = results[0].obb
    n_objects = len(obb.cls)
    for i in range(n_objects):
        obj_type = obb.cls[i]
        bbox_coords = obb.xyxyxyxyn[i].cpu().numpy()
        center = np.mean(bbox_coords, axis=0)
        world_center = -(center - 0.5) / 1.0834123569038272351866437016913
        # swap x and y
        world_x = world_center[1]
        world_y = world_center[0]
        rospy.loginfo(f"Object {i}: Type {obj_type}, Position ({bbox_coords}), Center {center}, World Position ({world_x}, {world_y})")
        if abs(world_x) < 0.23 and abs(world_y) < 0.23:
            rospy.loginfo(f"Object {i} is in the target area, picking it up")
            coord_pub = rospy.Publisher("/object_coordinates", Point, queue_size=10)
            coord_msg = Point(world_x, world_y, obj_type)  # Encode object type in z-coordinate
            coord_pub.publish(coord_msg)
            rospy.loginfo("Sent message to robot to pick up object")
            # Wait for the robot to pick up the object
            done = False
            while not done:
                msg = rospy.wait_for_message("/objects_placed", String)
                rospy.loginfo(f"Received message: {msg.data} - Object picked up")
                done = msg.data == "done"
    
    # Start the conveyor belt
    time.sleep(2)    
    conveyor_pub = rospy.Publisher("/conveyor_belt/cmd_vel", Twist, queue_size=10)
    start_msg = Twist()
    start_msg.linear.x = 0.05
    conveyor_pub.publish(start_msg)
    rospy.loginfo("Sent message to start conveyor belt")
    # Start spawning objects
    spawn_objects = True


def init_image_capture():
    bridge = CvBridge()
    # Load YOLO model
    # model_file = Path(__file__).parents[1] / "models" / "finetuned_yolo8obb_v2.pt"
    model_file = Path(__file__).parents[1] / "models" / "finetuned_yolo11n-obb.pt"
    try:
        yolo = YOLO(model_file)
        rospy.loginfo("YOLO model loaded")
    except Exception as e:
        rospy.logerr(f"Error loading YOLO model: {e}")
        raise
    # Callback function to capture image
    def image_callback(msg):
        global captured_image
        try:
            captured_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            rospy.loginfo(f"Captured image: Type {captured_image.__class__}, Shape {captured_image.shape}")
            if not spawn_objects:
                return
            # Perform object detection
            results = yolo(captured_image)
            obb = results[0].obb
            n_objects = len(obb.cls)
            if n_objects > 0:
                # Check y Coordinates of detected objects in the pictures
                # Only if at least one of them crossed the middle of the conveyor belt, stop the belt
                y_coords = obb.xyxyxyxyn[:, :, 1].cpu().numpy()
                lowest_y_coord = np.min(y_coords)
                rospy.loginfo(f"Detected {n_objects} objects, lowest y-coordinate: {lowest_y_coord}")
                if lowest_y_coord < 0.5:
                    pickup_objects_thread = Thread(target=pickup_objects, args=(yolo,))
                    pickup_objects_thread.start()
                
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    # Subscribe to the camera topic
    rospy.Subscriber('/camera1/image_raw', Image, image_callback)

def controller():
    rospy.init_node('controller_node', anonymous=True)
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    models_path = Path(__file__).parent.parent / "models"
    
    try:
        spawn_model_prox, blue_cube_xml, red_cylinder_xml = init_resources(models_path)
        init_image_capture()
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return
    
    try:
        time.sleep(2)
        coord_pub = rospy.Publisher("/object_coordinates", Point, queue_size=10)
        coord_msg = Point(0, 0, -1)  # Send fake message to initialize the topic
        coord_pub.publish(coord_msg)
        spawn_objects(spawn_model_prox, blue_cube_xml, red_cylinder_xml)
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def spawn_objects(spawn_model_prox, blue_cube_xml, red_cylinder_xml):
    
    conveyor_pub = rospy.Publisher("/conveyor_belt/cmd_vel", Twist, queue_size=10)
    start_msg = Twist()
    start_msg.linear.x = 0.05
    conveyor_pub.publish(start_msg)
    rospy.loginfo("Sent message to start conveyor belt")
    
    idx = 0
    while not rospy.is_shutdown():
        if spawn_objects:
            y_coord = random.uniform(-0.2, 0.2)
            # start with a cube, to work around weird cylinder physics at the beginning of the simulation
            if idx == 0:
                model_type, model_xml = "cube", blue_cube_xml
            model_type, model_xml = random.choice([("cube", blue_cube_xml), ("cylinder", red_cylinder_xml)])
            model_name = f"{model_type}_{idx}"
            model_pose = Pose(Point(-0.9, y_coord, 0.9251), Quaternion(0, 0, 0, 0))
            spawn_model_prox(model_name, model_xml, "", model_pose, "world")
            idx += 1
            rospy.loginfo(f"Spawned a new {model_type} at (-0.9, {y_coord:.2f}, 0.95)")
        time.sleep(5)


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass