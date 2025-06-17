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

# Object detection model setup
thres = 0.6
nmsThres = 0.3
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

# Takeoff
client.takeoffAsync().join()
client.moveToZAsync(-5, 2).join()
print("Drone airborne")

# Parameters
speed = 2
yaw_angle = 90
move_duration = 1
bbox_alert_min = 80
current_yaw = 0
altitude = -5
min_x, max_x = -50, 50
min_y, max_y = -50, 50
fps_limit = 10
last_frame_time = 0

# Lidar config tuning (increase density)
lidarData = client.getLidarData(lidar_name="LidarSensor1")
# Ensure in your AirSim `settings.json`:
# "NumberOfChannels": 32, "PointsPerSecond": 50000

# Random waypoint generator
def generate_random_waypoint():
    return random.uniform(min_x, max_x), random.uniform(min_y, max_y)

# Fetch image from drone
def get_drone_image():
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    if responses and responses[0].width != 0:
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
        return img_rgb.copy()
    else:
        return None

# Avoidance maneuver: back 5m, yaw +90°, forward 5m
def perform_avoidance():
    global current_yaw
    print("Obstacle detected — executing avoidance")

    client.moveByVelocityAsync(0, 0, 0, move_duration).join()

    # Move back
    vx = -speed * math.cos(math.radians(current_yaw))
    vy = -speed * math.sin(math.radians(current_yaw))
    client.moveByVelocityAsync(vx, vy, 0, 3).join()

    # Yaw turn
    current_yaw = (current_yaw + yaw_angle) % 360
    client.rotateToYawAsync(current_yaw).join()

    # Move forward
    vx = speed * math.cos(math.radians(current_yaw))
    vy = speed * math.sin(math.radians(current_yaw))
    client.moveByVelocityAsync(vx, vy, 0, 3).join()

    print("Avoidance maneuver complete")

# Move to waypoint incrementally with Lidar & collision monitoring
def move_to_waypoint(x_target, y_target):
    global current_yaw

    pos = client.getMultirotorState().kinematics_estimated.position
    x, y = pos.x_val, pos.y_val

    dx = x_target - x
    dy = y_target - y
    distance = math.hypot(dx, dy)
    angle_to_target = math.degrees(math.atan2(dy, dx))

    current_yaw = angle_to_target
    client.rotateToYawAsync(current_yaw).join()

    step_distance = 2
    steps = int(distance / step_distance)

    for _ in range(steps):
        # Lidar Obstacle Check
        lidarData = client.getLidarData(lidar_name="LidarSensor1")
        if len(lidarData.point_cloud) > 3: # type: ignore
            points = np.array(lidarData.point_cloud, dtype=np.float32).reshape(-1, 3)
            distances = np.linalg.norm(points, axis=1)
            min_dist = np.min(distances)
            if min_dist < 2:
                print(f"Lidar alert: Obstacle {min_dist:.2f} m ahead")
                perform_avoidance()
                return

        # Collision Check
        collision_info = client.simGetCollisionInfo()
        if collision_info.has_collided:
            print("Collision detected — avoidance triggered")
            perform_avoidance()
            return

        # Move small step
        vx = speed * math.cos(math.radians(current_yaw))
        vy = speed * math.sin(math.radians(current_yaw))
        client.moveByVelocityAsync(vx, vy, 0, 1).join()

# Main loop
control_enabled = True
try:
    while control_enabled:
        current_time = time.time()
        if current_time - last_frame_time < 1 / fps_limit:
            continue
        last_frame_time = current_time

        img = get_drone_image()
        if img is None:
            continue

        # Vision detection
        classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nmsThres)
        vision_alert = False
        if isinstance(classIds, np.ndarray) and len(classIds) != 0:
            for cId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
                x, y, w, h = box
                cvzone.cornerRect(img, box)
                label = f'{classNames[cId-1]} {round(conf*100,1)}%'
                cv2.putText(img, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,0,0), 2)
                if max(w, h) > bbox_alert_min:
                    print(f"Vision: {classNames[cId-1]} {conf*100:.1f}% detected")
                    vision_alert = True
                    break

        # Lidar/Collision alerts
        lidarData = client.getLidarData(lidar_name="LidarSensor1")
        lidar_alert = False
        if len(lidarData.point_cloud) > 3: # type: ignore
            points = np.array(lidarData.point_cloud, dtype=np.float32).reshape(-1, 3)
            distances = np.linalg.norm(points, axis=1)
            median_dist = np.median(distances)
            if median_dist < 2:
                print(f"Lidar alert: {median_dist:.2f} m")
                lidar_alert = True

        collision_detected = client.simGetCollisionInfo().has_collided

        if lidar_alert or vision_alert or collision_detected:
            perform_avoidance()
        else:
            waypoint_x, waypoint_y = generate_random_waypoint()
            print(f"Moving to waypoint: X={waypoint_x:.1f}, Y={waypoint_y:.1f}")
            move_to_waypoint(waypoint_x, waypoint_y)

        cv2.imshow("AirSim Drone Feed", img)
        cv2.waitKey(1)

        if keyboard.is_pressed('esc'):
            print("Landing...")
            client.landAsync().join()
            client.armDisarm(False)
            client.enableApiControl(False)
            control_enabled = False

finally:
    cv2.destroyAllWindows()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("Drone disarmed and API control released.")