import sys, time
import argparse
import cv2
import collections
from pymavlink import mavutil
import detector_mobilenet as detector
import vision
import keyboard

# Args parser
parser = argparse.ArgumentParser(description='Vision-based autonomous drone follow')
parser.add_argument('--debug_path', type=str, default="debug/run1", help='debug video output path')
parser.add_argument('--connection_string', type=str, default="/dev/ttyAMA0", help='MAVLink connection string')
args = parser.parse_args()

# Config
MAX_ALT = 2.5  # m
MIN_BOX_HEIGHT = 100  # pixels
MAX_BOX_HEIGHT = 250  # pixels
MA_X = collections.deque(maxlen=5)
STATE = "takeoff"

# Connect MAVLink
print("Connecting to drone via MAVLink...")
master = mavutil.mavlink_connection(args.connection_string, baud=115200)
master.wait_heartbeat()
print("Heartbeat received")

# Set flight mode to GUIDED
def set_mode(mode):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0, 1, mode, 0, 0, 0, 0, 0)
    print(f"Mode set to {mode}")

def arm_and_takeoff(target_altitude):
    print("Arming motors")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0)
    time.sleep(2)
    
    print("Takeoff")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, target_altitude)
    time.sleep(7)

# Control functions
def send_velocity(vx, vy, vz, yaw_rate):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111, 0, 0, 0, 
        vx, vy, vz,
        0, 0, 0,
        0, 0, yaw_rate)

# Visualize
def visualize(img):
    cv2.imshow("out", img)
    cv2.waitKey(1)

# Moving average calculation
def calculate_ma(ma_array):
    return sum(ma_array) / len(ma_array)

# Detector setup
detector.initialize_detector()
image_width, image_height = detector.get_image_size()
image_center = (image_width / 2, image_height / 2)
debug_writer = cv2.VideoWriter(args.debug_path + ".avi", cv2.VideoWriter_fourcc('M','J','P','G'), 25, (image_width, image_height))

def track():
    print("Tracking target")
    while True:
        if keyboard.is_pressed('q'):
            print("Manual stop")
            land()
        
        detections, fps, image = detector.get_detections()

        if len(detections) > 0:
            person = detections[0]
            person_center = person.Center

            x_delta = vision.get_single_axis_delta(image_center[0], person_center[0])
            MA_X.append(x_delta)
            x_delta_avg = calculate_ma(MA_X)

            # Estimate distance from bounding box height
            box_height = person.Bottom - person.Top
            if box_height < MIN_BOX_HEIGHT:
                vz = 0.5  # move forward
            elif box_height > MAX_BOX_HEIGHT:
                vz = -0.5  # move backward
            else:
                vz = 0.0  # hold position

            yaw_rate = x_delta_avg * 0.01  # tune proportional constant as needed

            send_velocity(vz, 0, 0, yaw_rate)

            cv2.rectangle(image, (int(person.Left), int(person.Bottom)), (int(person.Right), int(person.Top)), (0, 255, 0), 2)
            cv2.circle(image, (int(image_center[0]), int(image_center[1])), 5, (255, 0, 0), -1)
            cv2.circle(image, (int(person_center[0]), int(person_center[1])), 5, (0, 0, 255), -1)
            cv2.putText(image, f'Box Height: {box_height}px', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            visualize(image)
            debug_writer.write(image)
        else:
            print("No detection — hovering")
            send_velocity(0, 0, 0, 0)

def land():
    print("Landing")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)
    detector.close_camera()
    sys.exit(0)

# Mission flow
if __name__ == "__main__":
    set_mode(4)  # 4 = GUIDED mode for ArduCopter
    time.sleep(1)
    arm_and_takeoff(MAX_ALT)
    track()
