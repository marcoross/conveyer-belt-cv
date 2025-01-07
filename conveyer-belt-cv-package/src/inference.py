from ultralytics import YOLO
import os
import math

# Path to the model and image
model_path = os.path.join("..", "models", "yolov5s.pt")  # Path to the model (no change needed)
image_path = os.path.join("..","data", "images", "test", "camera_image_5.png")  # Corrected path to the image

# Load the YOLOv5 model
# model = YOLO(model_path)

# Load the fine-tuned model
model = YOLO("../../runs/detect/custom_yolo_model/weights/best.pt")

# Run inference on the image
results = model(image_path)

# Since results is a list, get the first item in the list
result = results[0]  # Access the first result

# Show results (image with bounding boxes)
result.show()  # Now call .show() on the result

# Optionally, print the results in a pandas DataFrame format
# print(result.pandas().xywh)

# Extract bounding box information
boxes = result.boxes
for box in boxes:
    coords = box.xyxy[0]  # [x_min, y_min, x_max, y_max]
    confidence = box.conf[0]  # Confidence score
    cls = box.cls[0]  # Class label index

    print(f"Coordinates: {coords}")
    print(f"Confidence: {confidence}")
    print(f"Class: {cls}")


def transform_coordinates(x1, y1, x2, y2, x3, y3, x4, y4):
    dx = x2 - x1
    dy = y2 - y1
    # Angle in range [-pi, pi]
    theta = math.atan2(dy, dx)
    # Normalize angle in range [-pi/4, pi/4]
    normalized_theta = (theta + math.pi / 4) % (math.pi / 2) - math.pi / 4

    x_center = (x1 + x2 + x3 + x4) / 4
    object_y = -1/1.083412 * (x_center - 0.5)
    y_center = (y1 + y2 + y3 + y4) / 4
    object_x = -1/1.083412 * (y_center - 0.5)

    return object_x, object_y, normalized_theta
