import sys, time, argparse
import collections
import cv2
import keyboard

from modules import detector_mobilenet as detector
from modules import camera
from modules import vision
from modules import control

# ----- Argument Parsing -----
parser = argparse.ArgumentParser(description='Object Following Drone - Raspberry Pi Version')
parser.add_argument('--debug_path', type=str, default="debug/run1", help='debug message name')
parser.add_argument('--mode', type=str, default='flight', help='flight or test (simulator)')
parser.add_argument('--control', type=str, default='PID', help='PID or P controller')
args = parser.parse_args()

# ----- Constants -----
MAX_FOLLOW_DIST = 1.5  # meters
MAX_ALT = 2.0          # meters
MAX_MA_X_LEN = 5
MAX_MA_Z_LEN = 5

MA_X = collections.deque(maxlen=MAX_MA_X_LEN)
MA_Z = collections.deque(maxlen=MAX_MA_Z_LEN)
STATE = "takeoff"

# ----- Setup Phase -----
def setup():
    print("Setting up detector...")
    detector.initialize_detector()

    print("Initializing Pi camera...")
    camera.create_camera(0)
    
    print("Connecting to drone...")
    if args.mode == "flight":
        control.connect_drone('/dev/ttyACM0')  # USB telemetry
    else:
        control.connect_drone('127.0.0.1:14550')  # SITL
    
    control.set_flight_altitude(MAX_ALT)
    control.configure_PID(args.control)
    control.initialize_debug_logs(args.debug_path)

setup()

image_width, image_height = camera.get_image_size(0)
image_center = (image_width / 2, image_height / 2)

fourcc = int(getattr(cv2, 'VideoWriter_fourcc')(*'MJPG'))


debug_image_writer = cv2.VideoWriter(args.debug_path + ".avi", fourcc, 25, (image_width, image_height))

# ----- FSM States -----
def track():
    print("State: TRACKING")
    while True:
        if keyboard.is_pressed('q'):
            print("Manual exit - landing")
            return "land"

        detections, fps, image = detector.get_detections()
        if detections:
            person = detections[0]
            person_center = person.Center
            bbox_height = person.Bottom - person.Top

            est_dist = vision.estimate_distance_from_box(bbox_height, image_height)
            z_delta = est_dist - MAX_FOLLOW_DIST
            x_delta = vision.get_single_axis_delta(image_center[0], person_center[0])

            MA_X.append(x_delta)
            MA_Z.append(z_delta)

            if len(MA_Z) > 0:
                z_delta_MA = sum(MA_Z) / len(MA_Z)
                control.setZDelta(z_delta_MA)

            if len(MA_X) > 0:
                x_delta_MA = sum(MA_X) / len(MA_X)
                control.setXdelta(x_delta_MA)

            control.control_drone()

            visualize(image, person, float(fps), float(z_delta), float(x_delta), float(est_dist))
        else:
            return "search"

def search():
    print("State: SEARCHING")
    control.stop_drone()
    start_time = time.time()
    while time.time() - start_time < 40:
        if keyboard.is_pressed('q'):
            return "land"
        detections, fps, image = detector.get_detections()
        if detections:
            return "track"
        visualize(image)
    return "land"

def takeoff():
    control.print_drone_report()
    print("State: TAKEOFF")
    control.arm_and_takeoff(MAX_ALT)
    return "search"

def land():
    print("State: LAND")
    control.land()
    camera.close_cameras()
    cv2.destroyAllWindows()
    sys.exit(0)

def visualize(img, person=None, fps: float = 0.0, z_delta: float = 0.0, x_delta: float = 0.0, distance: float = 0.0):
    if person:
        cv2.rectangle(img, (person.Left, person.Top), (person.Right, person.Bottom), (0, 0, 255), 2)
        cv2.circle(img, (int(image_center[0]), int(image_center[1])), 10, (0, 255, 0), -1)
        cv2.circle(img, person.Center, 10, (255, 0, 0), -1)
        cv2.putText(img, f"FPS: {fps:.2f}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(img, f"Z Delta: {z_delta:.2f}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(img, f"X Delta: {x_delta:.2f}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(img, f"Distance: {distance:.2f}m", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    if args.mode == "flight":
        debug_image_writer.write(img)
    else:
        cv2.imshow("Tracking", img)
        cv2.waitKey(1)

# ----- Main FSM Loop -----
while True:
    if STATE == "track":
        control.set_system_state("track")
        STATE = track()
    elif STATE == "search":
        control.set_system_state("search")
        STATE = search()
    elif STATE == "takeoff":
        STATE = takeoff()
    elif STATE == "land":
        STATE = land()