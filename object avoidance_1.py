import airsim
import cv2
import numpy as np
import cvzone
import time
import keyboard
import math
import random

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
print("Connected to AirSim")

# Object detection model
thres = 0.55
nmsThres = 0.2
classFile = 'coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().rstrip('\n').split('\n')
configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'frozen_inference_graph.pb'
net = cv2.dnn_DetectionModel(weightsPath, configPath) # type: ignore
net.setInputSize(224, 224)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# Take off
client.takeoffAsync().join()
client.moveToZAsync(-5, 2).join()
print("Drone is airborne")

# Movement parameters
speed = 2
yaw_angle = 90
move_duration = 1
bbox_alert_min = 80
current_yaw = 0

# Flight box limits (in meters)
min_x, max_x = -50, 50
min_y, max_y = -50, 50
altitude = -5  # constant altitude

# Get image from drone
def get_drone_image(client):
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    if responses and responses[0].width != 0:
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
        return img_rgb.copy()
    else:
        return None

# Generate random waypoint inside bounds
def generate_random_waypoint():
    x = random.uniform(min_x, max_x)
    y = random.uniform(min_y, max_y)
    return x, y

# Move to waypoint relative to world frame
def move_to_waypoint(x, y):
    client.moveToPositionAsync(x, y, altitude, speed,
        drivetrain=airsim.DrivetrainType.ForwardOnly,
        yaw_mode=airsim.YawMode(False, 0)).join()

control_enabled = True

try:
    while control_enabled:
        img = get_drone_image(client)

        # Get collision info
        collision_info = client.simGetCollisionInfo()
        collision_detected = collision_info.has_collided

        object_detected = False

        if img is not None:
            classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nmsThres)
            if len(classIds) != 0:
                for cId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
                    x, y, w, h = box
                    cvzone.cornerRect(img, box)
                    label = f'{classNames[cId-1]} {round(conf*100,1)}%'
                    cv2.putText(img, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)

                    if max(w, h) > bbox_alert_min:
                        object_detected = True
                        print(f"\nDetected '{classNames[cId-1]}' nearby. Avoiding...")

                        # Stop
                        client.moveByVelocityAsync(0, 0, 0, move_duration).join()

                        # Move back 10 m relative to heading
                        vx = -speed * math.cos(math.radians(current_yaw))
                        vy = -speed * math.sin(math.radians(current_yaw))
                        client.moveByVelocityAsync(vx, vy, 0, 5, 
                            drivetrain=airsim.DrivetrainType.ForwardOnly,
                            yaw_mode=airsim.YawMode(False, 0)).join()

                        # Yaw turn
                        current_yaw += yaw_angle
                        if current_yaw >= 360:
                            current_yaw -= 360
                        client.rotateToYawAsync(current_yaw).join()

                        # Move forward 10 m relative to new heading
                        vx = speed * math.cos(math.radians(current_yaw))
                        vy = speed * math.sin(math.radians(current_yaw))
                        client.moveByVelocityAsync(vx, vy, 0, 5,
                            drivetrain=airsim.DrivetrainType.ForwardOnly,
                            yaw_mode=airsim.YawMode(False, 0)).join()

                        print("Avoidance maneuver complete.")
                        break  # Only one object handled per frame

            cv2.imshow("AirSim Drone Feed", img)
            cv2.waitKey(1)

        if object_detected or collision_detected:
            if collision_detected:
                print("\nCollision detected by AirSim sensors. Executing avoidance.")
            # Skip moving to waypoint this iteration after handling
            continue

        # Move to random waypoint
        waypoint_x, waypoint_y = generate_random_waypoint()
        print(f"Moving to waypoint: X={waypoint_x:.1f}, Y={waypoint_y:.1f}")
        move_to_waypoint(waypoint_x, waypoint_y)

        if keyboard.is_pressed('esc'):
            print("\n🛬 Landing...")
            client.landAsync().join()
            client.armDisarm(False)
            client.enableApiControl(False)
            control_enabled = False

        time.sleep(0.1)

finally:
    cv2.destroyAllWindows()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("Drone disarmed and API control released.")
