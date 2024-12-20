from ultralytics import YOLO
import os

# Path to the model and image
model_path = os.path.join("..", "models", "yolov5s.pt")  # Path to the model (no change needed)
image_path = os.path.join("..","data", "test_images", "image1.png")  # Corrected path to the image

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
