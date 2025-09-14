from roboflow import Roboflow
from ultralytics import YOLO

# Step 1: Download dataset from Roboflow
rf = Roboflow(api_key="lro3ndkviMuzv1NJEXxb")
project = rf.workspace("charak").project("exact-top-surface-of-blue-box")
dataset = project.version(2).download("yolov8")

# Step 2: Load YOLO model
model = YOLO("yolov8n-seg.pt")

# Step 3: Train the model using the downloaded dataset path
model.train(
    data=f"{dataset.location}/data.yaml",
    epochs=50,
    imgsz=640
)
