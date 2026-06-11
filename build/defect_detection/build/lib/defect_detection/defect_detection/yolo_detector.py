from ultralytics import YOLO
import yaml
from spot_cam_loading import image_publisher
import numpy as np

with open ('dataset.yaml', 'r') as f:
    data = yaml.safe_load(f)
    classes = list(data['names'])

COLORS = np.random.uniform(0, 255, size=(len(classes), 3))
model = YOLO('yolov11m.engine')

def detect_defects():
    results = model(image_publisher.get_image_data())
    return results