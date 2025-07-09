from flask import Flask, Response
import cv2
import subprocess

app = Flask(__name__)

def generate_frames():
    # Start libcamera-vid subprocess and capture its output
    cmd = ['libcamera-vid', '--inline', '-t', '0', '--width', '640', '--height', '480', '--codec', 'mjpeg', '-o', '-']
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE)

    while True:
        frame = process.stdout.read(1024)
        if not frame:
            break
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return "<h2>Pi Camera Live Feed</h2><img src='/video_feed'/>"

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True)

