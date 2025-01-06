import os
from ultralytics import YOLO

# Define paths
model_path = os.path.join('..', 'models', 'yolo11s-obb.pt')
input_folder = os.path.join('..', 'data', 'images', 'test')  
output_folder = os.path.join('..', 'tests', 'yolo11s_obb')

# Create the output folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Load YOLO model
model = YOLO(model_path)

# Get all image files in the input folder (e.g., .png, .jpg)
image_files = [f for f in os.listdir(input_folder) if f.lower().endswith(('.png', '.jpg', '.jpeg'))]

# Loop through all images in the folder
for image_file in image_files:
    input_image_path = os.path.join(input_folder, image_file)  
    print(f"Processing {input_image_path}")
    
    # Run inference on the current image
    results = model(input_image_path, save=True, save_dir=output_folder)
    
    # Optionally, print or log results for each image
    print(f"Results saved to: {output_folder}")


