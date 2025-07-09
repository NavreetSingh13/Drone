# detector_mobilenet.py
# --------------------------------------------
# Uses MobileNet SSD with OpenCV DNN module
# --------------------------------------------

import cv2
import numpy as np
from modules import camera

# Paths to your model files
MODEL_PATH = "models/frozen_inference_graph.pb"
CONFIG_PATH = "models/ssd_mobilenet_v2_coco_2018_03_29.pbtxt"

# Load the model once
net = None

def initialize_detector():
    global net
    print("[INFO] Loading MobileNet SSD model...")
    net = cv2.dnn.readNetFromTensorflow(MODEL_PATH, CONFIG_PATH)
    camera.create_camera(0)
    print("[INFO] Detector initialized.")

def get_detections():
    """
    Returns detections, fps, and frame.
    """
    if net is None:
        raise RuntimeError("Detector not initialized")

    frame = camera.get_video(0)
    blob = cv2.dnn.blobFromImage(frame, size=(300, 300), swapRB=True, crop=False)
    net.setInput(blob)

    start = cv2.getTickCount()
    output = net.forward()
    end = cv2.getTickCount()
    fps = cv2.getTickFrequency() / (end - start)

    h, w = frame.shape[:2]
    detections = []
    for detection in output[0, 0, :, :]:
        confidence = float(detection[2])
        class_id = int(detection[1])

        if class_id == 1 and confidence > 0.5:  # person class
            left = int(detection[3] * w)
            top = int(detection[4] * h)
            right = int(detection[5] * w)
            bottom = int(detection[6] * h)
            detections.append(Detection(left, top, right, bottom))

    return detections, round(fps, 2), frame

class Detection:
    def __init__(self, left, top, right, bottom):
        self.Left = left
        self.Top = top
        self.Right = right
        self.Bottom = bottom
        self.Center = ((left + right) // 2, (top + bottom) // 2)
