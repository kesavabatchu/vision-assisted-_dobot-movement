import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO
import os
import  time
img_path=r'C:/btp/python files/images/test1.jpg'
model = YOLO("yolov8n.pt")  # Model file in your main folder
results = model(img_path, save=True)
for x in results:
    x.show()
    