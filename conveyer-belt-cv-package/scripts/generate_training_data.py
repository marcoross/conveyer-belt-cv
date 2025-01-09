#!/usr/bin/python
from __future__ import annotations
import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose, Point, Twist, Quaternion
from sensor_msgs.msg import Image
import time
from pathlib import Path
import random
from typing import NamedTuple
from cv_bridge import CvBridge, CvBridgeError
import cv2
from csv import DictWriter


class ObjectDescription(NamedTuple):
    name: str
    type: str
    x: float
    y: float
    angle: float
    
    def too_close_to(self, other: ObjectDescription) -> bool:
        distance = ((self.x - other.x) ** 2 + (self.y - other.y) ** 2) ** 0.5
        return distance < 0.1
    
    def too_close_to_any(self, others: list[ObjectDescription]) -> bool:
        for other in others:
            if self.too_close_to(other):
                return True
        return False

def generate_training_data():
    rospy.init_node('generate_training_data_node', anonymous=True)
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    rospy.wait_for_service('/gazebo/delete_model')
    
    models_path = Path(__file__).parent.parent / "models"
    
    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        
        with open(models_path / "small_blue_cube" / "model.sdf", "r") as f:
            blue_cube_xml = f.read()
            
        with open(models_path / "small_red_cylinder" / "model.sdf", "r") as f:
            red_cylinder_xml = f.read()
        
        
        images_path = Path(__file__).parent.parent / 'images'
        images_path.mkdir(exist_ok=True)
        bridge = CvBridge()
        
        # Callback function to capture image
        captured_image = None
        def image_callback(msg):
            global captured_image
            try:
                captured_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error: {0}".format(e))

        # Subscribe to the camera topic
        image_sub = rospy.Subscriber('/camera1/image_raw', Image, image_callback)

        # Function to capture a new image
        def capture_image():
            global captured_image
            captured_image = None
            while captured_image is None:
                rospy.sleep(0.1)
            return captured_image
        
        object_info_csv = images_path / "object_info.csv"
        with open(object_info_csv, "w") as f:
            writer = DictWriter(f, fieldnames=["image_name", "object_type", "object_x", "object_y", "object_angle"])
            writer.writeheader()
        
        time.sleep(5)
        
        idx = 0
        current_models: list[ObjectDescription] = []
        while not rospy.is_shutdown():
            # Remove all objects from the simulation
            for model in current_models:
                delete_model_prox(model.name)
                rospy.loginfo(f"Removed {model_name}")
            current_models.clear()
            
            # Stop the conveyor belt
            conveyor_pub = rospy.Publisher("/conveyor_belt/cmd_vel", Twist, queue_size=10)
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            conveyor_pub.publish(stop_msg)
            rospy.loginfo("Sent message to stop conveyor belt")
            
            # Put new objects on the conveyor belt
            n_objects = random.randint(1, 6)
            for _ in range(n_objects):
                y_coord = random.uniform(-0.2, 0.2)
                x_coord = random.uniform(-0.2, 0.2)
                angle = random.uniform(-0.78539816339, 0.78539816339)
                model_type, model_xml = random.choice([("cube", blue_cube_xml), ("cylinder", red_cylinder_xml)])
                if model_type == "cylinder":
                    angle = 0.0
                model_name = f"{model_type}_{idx}"
                model_pose = Pose(Point(x_coord, y_coord, 0.95), Quaternion(0.0, 0.0, angle, 1.0))
                desc = ObjectDescription(model_name, model_type, x_coord, y_coord, angle)
                if desc.too_close_to_any(current_models):
                    continue
                spawn_model_prox(model_name, model_xml, "", model_pose, "world")
                current_models.append(desc)
                idx += 1
                rospy.loginfo(f"Spawned a new {model_type} at (-0.9, {y_coord:.2f}, 0.95)")
            
            time.sleep(2)
            
            cv_image = capture_image()

            # Save the image
            n_images_in_folder = len(list(images_path.glob('*.png')))
            image_name = f"camera_image_{n_images_in_folder}.png"
            rospy.loginfo("Image received!")
            cv2.imwrite(str(images_path / image_name), cv_image)
            with open(object_info_csv, "a") as f:
                writer = DictWriter(f, fieldnames=["image_name", "object_type", "object_x", "object_y", "object_angle"])
                for model in current_models:
                    writer.writerow({"image_name": image_name, "object_type": model.type, "object_x": model.x, "object_y": model.y, "object_angle": model.angle})
            
            
            time.sleep(2)
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    try:
        generate_training_data()
    except rospy.ROSInterruptException:
        pass