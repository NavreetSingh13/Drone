from flask import Flask, Response
import torch
import cv2
import numpy as np
import subprocess
import threading
import sys
import os

app = Flask(__name__)

# Load YOLOv5 model
model = torch.hub.load('yolov5', 'custom', path='yolov5s.pt', source='local')
model.conf = 0.4  # Confidence threshold

# Global camera process
process = None
data_buffer = b""

def start_camera_process():
    global process
    cmd = [
        'libcamera-vid', '--inline', '--nopreview', '-t', '0',
        '--width', '640', '--height', '480', '--codec', 'mjpeg', '-o', '-'
    ]
    try:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        print("[INFO] libcamera-vid process started")
        threading.Thread(target=check_process_errors, daemon=True).start()
    except Exception as e:
        print(f"[ERROR] Failed to start libcamera-vid: {e}")
        sys.exit(1)

def check_process_errors():
    global process
    for line in process.stderr:
        decoded = line.decode('utf-8').strip()
        if "ERROR" in decoded:
            print(f"[libcamera-vid ERROR] {decoded}")
            print("[FATAL] Camera could not be acquired. Exiting.")
            process.kill()
            sys.exit(1)

def read_camera_stream():
    global data_buffer
    while True:
        if process:
            chunk = process.stdout.read(4096)
            if chunk:
                data_buffer += chunk

start_camera_process()
threading.Thread(target=read_camera_stream, daemon=True).start()

def generate_frames():
    global data_buffer
    while True:
        start = data_buffer.find(b'\xff\xd8')
        end = data_buffer.find(b'\xff\xd9')
        if start != -1 and end != -1 and end > start:
            jpg = data_buffer[start:end+2]
            data_buffer = data_buffer[end+2:]

            img_array = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # Run YOLOv5 inference
            results = model(frame)

            # Annotate the frame with detection results
            annotated = np.squeeze(results.render())

            # Encode and yield
            ret, buffer = cv2.imencode('.jpg', annotated)
            if not ret:
                continue

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '''
    <html>
    <head>
        <title>Drone Object Detection Feed</title>
        <style>
            body { text-align: center; background: #111; color: white; }
            img { border: 3px solid #555; margin-top: 20px; }
        </style>
    </head>
    <body>
        <h2>Live Drone Object Detection</h2>
        <img src="/video_feed" width="640" height="480">
    </body>
    </html>
    '''

if __name__ == '__main__':
    print("[INFO] Starting Flask server at http://<your_pi_ip>:5001")
    app.run(host='0.0.0.0', port=5001, threaded=True)
