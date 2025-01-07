import os
import pandas as pd
import shutil
from sklearn.model_selection import train_test_split
from ultralytics import YOLO

source_folder = "../data/images/source"
train_folder = "../data/images/train"
val_folder = "../data/images/val"
test_folder = "../data/images/test"
csv_file = "../data/object_info.csv"
train_labels_folder = "../data/labels/train"
val_labels_folder = "../data/labels/val"


def create_data():
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


def create_labels():
    # Create labels folder if it doesn't exist
    for folder in [train_labels_folder, val_labels_folder]:
        os.makedirs(folder, exist_ok=True)

    class_mapping = {"cube": 0, "cylinder": 1}
    data = pd.read_csv(csv_file)

    for image_name, group in data.groupby("image_name"):
        if os.path.exists(os.path.join(train_folder, image_name)):
            label_path = os.path.join(train_labels_folder, f"{os.path.splitext(image_name)[0]}.txt")
        elif os.path.exists(os.path.join(val_folder, image_name)):
            label_path = os.path.join(val_labels_folder, f"{os.path.splitext(image_name)[0]}.txt")
        else:
            print(f"Image {image_name} not found in train or val folders. Skipping...")
            continue

        with open(label_path, "w") as label_file:
            for _, row in group.iterrows():
                class_id = class_mapping[row["object_type"]]
                # Convert ROS coordinates [-0.5, 0,5] to YOLO format [0, 1]
                x_center, y_center, width, height = 0.5 - row["object_y"], 0.5 - row["object_x"], 0.08, 0.08
                label_file.write(f"{class_id} {x_center} {y_center} {width} {height}\n")


def train_model():
    model_path = os.path.join("..", "models", "yolov5s.pt")
    model = YOLO(model_path)

    model.train(
        data="data.yaml",
        epochs=5,
        imgsz=640,
        batch=16,
        workers=4,
        name="custom_yolo_model"
    )

    metrics = model.val(data="data.yaml")  # Run evaluation
    print(metrics)


# create_data()
# create_labels()
train_model()