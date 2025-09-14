from ultralytics import YOLO

model = YOLO("yolov8n-seg.pt")  # Use your segmentation model
print(model.names)
