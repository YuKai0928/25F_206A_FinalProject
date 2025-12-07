import torch
import urllib.request
import os

model_path = "box_detector_yolov5n.pt"
# os.makedirs(os.path.dirname(model_path), exist_ok=True)

# Download YOLOv5s pretrained model
url = "https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5s.pt"
urllib.request.urlretrieve(url, model_path)