from ultralytics import YOLO
import os

# Path to the model and image
model_path = os.path.join("..", "models", "yolov5s.pt")  # Path to the model (no change needed)
image_path = os.path.join("..","data", "images", "car.png")  # Corrected path to the image

# Load the YOLOv5 model
model = YOLO(model_path)

# Run inference on the image
results = model(image_path)

# Since results is a list, get the first item in the list
result = results[0]  # Access the first result

# Show results (image with bounding boxes)
result.show()  # Now call .show() on the result

# Optionally, print the results in a pandas DataFrame format
print(result.pandas().xywh)