# camera.py
# -------------------------------------------
# Uses camera_stream.py for Pi Camera feed
# -------------------------------------------

import requests
import numpy as np
import cv2

# URL of the running camera_stream.py server
STREAM_URL = "http://localhost:5000/video_feed"

def create_camera(camera_id=0):
    print("[INFO] Camera initialized via camera stream at", STREAM_URL)

def get_video(camera_id=0):
    """
    Reads a single frame from the Flask camera stream.
    """
    resp = requests.get(STREAM_URL, stream=True)
    if resp.status_code != 200:
        raise RuntimeError("Failed to connect to camera stream")

    bytes_ = b""
    for chunk in resp.iter_content(chunk_size=1024):
        bytes_ += chunk
        a = bytes_.find(b'\xff\xd8')
        b_ = bytes_.find(b'\xff\xd9')
        if a != -1 and b_ != -1:
            jpg = bytes_[a:b_+2]
            bytes_ = bytes_[b_+2:]
            img_array = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            return frame

def get_image_size(camera_id=0):
    # Returns default stream resolution
    return 640, 480

def close_cameras():
    print("[INFO] Camera stream closed.")
