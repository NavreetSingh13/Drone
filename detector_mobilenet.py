import cv2
import numpy as np

class Detector:
    def __init__(self):
        self.net = None
        self.image_width = 640
        self.image_height = 480
        self.CONFIDENCE_THRESHOLD = 0.5

        # Model files (ensure these exist)
        self.MODEL_PATH = "models/frozen_inference_graph.pb"
        self.CONFIG_PATH = "models/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"

    def initialize(self):
        self.net = cv2.dnn.readNetFromTensorflow(self.MODEL_PATH, self.CONFIG_PATH)
        print("Detector initialized")

    def get_image_size(self):
        return self.image_width, self.image_height

    def get_detections(self):
        if self.net is None:
            raise RuntimeError("Detector not initialized. Call initialize() first.")

        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        ret, frame = cap.read()
        cap.release()

        if not ret:
            raise RuntimeError("Could not read from Pi Camera")

        blob = cv2.dnn.blobFromImage(frame, size=(300, 300), swapRB=True, crop=False)
        self.net.setInput(blob)

        start = cv2.getTickCount()
        output = self.net.forward()
        end = cv2.getTickCount()
        fps = cv2.getTickFrequency() / (end - start)

        h, w = frame.shape[:2]
        detections = []
        for detection in output[0, 0, :, :]:
            confidence = float(detection[2])
            class_id = int(detection[1])

            if class_id == 1 and confidence > self.CONFIDENCE_THRESHOLD:  # person
                left = int(detection[3] * w)
                top = int(detection[4] * h)
                right = int(detection[5] * w)
                bottom = int(detection[6] * h)
                detections.append(Detection(left, top, right, bottom))

        return detections, round(fps, 2), frame

# Helper class for bounding box structure (like JetsonNet)
class Detection:
    def __init__(self, left, top, right, bottom):
        self.Left = left
        self.Top = top
        self.Right = right
        self.Bottom = bottom
        self.Center = ((left + right) // 2, (top + bottom) // 2)

# Global interface (compatible with your original code)
detector_instance = Detector()

def initialize_detector():
    detector_instance.initialize()

def get_image_size():
    return detector_instance.get_image_size()

def get_detections():
    return detector_instance.get_detections()

def close_camera():
    pass  # no persistent camera to release