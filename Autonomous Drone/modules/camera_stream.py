from flask import Flask, Response
import subprocess
import threading
import sys
import cv2
import numpy as np

app = Flask(__name__)

process = None
data_buffer = b""

# Load Haar cascade for face detection (included with OpenCV)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + "haarcascade_frontalface_default.xml")

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
        elif "WARN" in decoded:
            print(f"[libcamera-vid WARNING] {decoded}")
        elif "INFO" in decoded:
            print(f"[libcamera-vid INFO] {decoded}")

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

            # Decode JPEG to OpenCV image
            img_array = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if frame is None:
                continue

            # Convert to grayscale for Haar detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Perform detection
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            # Draw detections
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, "Face", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

            # Encode frame back to JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
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
        <title>Pi Camera Live Detection Feed</title>
        <style>
            body { text-align: center; background: #222; color: white; }
            img { border: 2px solid #444; }
        </style>
    </head>
    <body>
        <h2>Pi Camera Live Detection Feed</h2>
        <img src="/video_feed" width="640" height="480">
    </body>
    </html>
    '''

if __name__ == "__main__":
    print("[INFO] Starting Flask server on http://<pi_ip>:5001 ...")
    app.run(host='0.0.0.0', port=5001, threaded=True)

