#This code was trained on google collabs. You can find the link on the project here:
# -*- coding: utf-8 -*-
!nvidia-smi

import os
HOME = os.getcwd()
print(HOME)

## Install YOLOv8

!pip install ultralytics -q

import ultralytics
ultralytics.checks()

"""Now, we can import YOLOv8 into our Notebook:"""

from ultralytics import YOLO

from IPython.display import display, Image

### Step 5: Export dataset


# Commented out IPython magic to ensure Python compatibility.
!mkdir {HOME}/datasets
# %cd {HOME}/datasets

!pip install roboflow --quiet

import roboflow

roboflow.login()

from roboflow import Roboflow
rf = Roboflow(api_key="yAyyw9rHq1kn3z0ercUE")
project = rf.workspace("roboticsproject-beet5").project("robotics-3pq1b")
version = project.version(4)
dataset = version.download("yolov8-obb")

import yaml

with open(f'{dataset.location}/data.yaml', 'r') as file:
    data = yaml.safe_load(file)

data['path'] = dataset.location

with open(f'{dataset.location}/data.yaml', 'w') as file:
    yaml.dump(data, file, sort_keys=False)

"""## Train a YOLOv8 OBB Object Detection Model

With our dataset downloaded, we can now train a YOLOv8 OBB object detection model. 
"""

from ultralytics import YOLO

model = YOLO('yolov8n-obb.pt')

results = model.train(data=f"{dataset.location}/data.yaml", epochs=100, imgsz=640)
