from ultralytics import YOLO
import os
import shutil
from sklearn.model_selection import train_test_split

def create_data():
    source_folder = "../data/images/source"
    train_folder = "../data/images/train"
    val_folder = "../data/images/val"
    test_folder = "../data/images/test"

    # Create destination folders if they don't exist
    for folder in [train_folder, val_folder, test_folder]:
        os.makedirs(folder, exist_ok=True)

    train_ratio = 0.8
    validation_ratio = 0.18
    test_ratio = 0.02

    images = [f for f in os.listdir(source_folder) if os.path.isfile(os.path.join(source_folder, f))]
    assert train_ratio + validation_ratio + test_ratio == 1, "Ratios must add up to 1"

    train_images, temp_images = train_test_split(images, test_size=(1 - train_ratio), random_state=1)
    val_images, test_images = train_test_split(temp_images, test_size=test_ratio / (test_ratio + validation_ratio), random_state=1)

    # Move images to their respective folders
    def move_images(image_list, destination_folder):
        for image in image_list:
            shutil.move(os.path.join(source_folder, image), os.path.join(destination_folder, image))

    move_images(train_images, train_folder)
    move_images(val_images, val_folder)
    move_images(test_images, test_folder)


def train_model():
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


create_data()
# train_model()