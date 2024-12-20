from ultralytics import YOLO
import os

model_path = os.path.join("..", "models", "yolov5s.pt")
model = YOLO(model_path)

model.train(
    data="data.yaml",
    epochs=20,
    imgsz=640,
    batch=16,
    workers=4,
    name="custom_yolo_model"
)

metrics = model.val(data="data.yaml")  # Run evaluation
print(metrics)
