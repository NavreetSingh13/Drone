import airsim
import cv2
import numpy as np
import time
import math
import random

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
print("[✔] Connected to AirSim")

# Takeoff
client.takeoffAsync().join()
client.moveToZAsync(-5, 3).join()
print("[✔] Drone airborne")

# Parameters
fps = 10
focal_length = 320  # in pixels (assuming image width 640)
baseline = 0.5      # in meters (distance between cameras)
speed = 3
altitude = -5
min_x, max_x = -50, 50
min_y, max_y = -50, 50

# Load object detection model
net = cv2.dnn_DetectionModel('frozen_inference_graph.pb', 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt') # type: ignore
net.setInputSize(320, 320)
net.setInputScale(1 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

def generate_random_waypoint():
    return random.uniform(min_x, max_x), random.uniform(min_y, max_y)

# Move to random waypoint
def move_to_random_waypoint():
    x, y = generate_random_waypoint()
    print(f"➡ Moving to random waypoint: X={x:.1f}, Y={y:.1f}")
    client.moveToPositionAsync(x, y, altitude, speed).join()

# Start by moving to one waypoint
move_to_random_waypoint()

# Main loop
try:
    while True:
        start_time = time.time()

        # Get stereo images
        responses = client.simGetImages([
            airsim.ImageRequest("front_left", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("front_right", airsim.ImageType.Scene, False, False)
        ])

        if len(responses) < 2 or responses[0].width == 0 or responses[1].width == 0:
            continue

        left_img = np.frombuffer(responses[0].image_data_uint8, np.uint8).reshape(
            responses[0].height, responses[0].width, 3)
        right_img = np.frombuffer(responses[1].image_data_uint8, np.uint8).reshape(
            responses[1].height, responses[1].width, 3)

        # Detect object in left image
        classIds, confs, boxes = net.detect(left_img, confThreshold=0.6, nmsThreshold=0.3)
        if len(classIds) > 0:
            for i in range(len(classIds)):
                c, conf, box = classIds[i], confs[i], boxes[i]
                x, y, w, h = box
                cv2.rectangle(left_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

                cx_l = x + w // 2
                cy_l = y + h // 2

                # Naive block matching for disparity
                disparity = None
                best_sim = -1
                patch_size = 20
                patchL = cv2.cvtColor(left_img[max(0, cy_l-patch_size//2):cy_l+patch_size//2,
                                               max(0, cx_l-patch_size//2):cx_l+patch_size//2], cv2.COLOR_BGR2GRAY)

                for offset in range(0, 50):
                    cx_r = cx_l - offset
                    if cx_r < 0 or cx_r+patch_size >= right_img.shape[1]:
                        continue
                    patchR = cv2.cvtColor(right_img[max(0, cy_l-patch_size//2):cy_l+patch_size//2,
                                                    max(0, cx_r-patch_size//2):cx_r+patch_size//2], cv2.COLOR_BGR2GRAY)

                    if patchR.shape != patchL.shape or patchR.size == 0:
                        continue

                    sim = cv2.matchTemplate(patchR, patchL, cv2.TM_CCOEFF_NORMED)[0][0]
                    if sim > best_sim:
                        best_sim = sim
                        disparity = offset

                if disparity and disparity != 0:
                    distance = focal_length * baseline / disparity
                    print(f"Object ID {int(c)} | Disparity: {disparity}px | Distance: {distance:.2f}m")

                    # Get drone global position
                    state = client.getMultirotorState()
                    drone_pos = state.kinematics_estimated.position
                    print(f"Drone Position: x={drone_pos.x_val:.1f}, y={drone_pos.y_val:.1f}, z={drone_pos.z_val:.1f}")

                break  # Only track first detection

        # Show frames
        cv2.imshow("Left Camera", left_img)
        cv2.imshow("Right Camera", right_img)
        if cv2.waitKey(1) & 0xFF == 27:
            break

        # Simulate FPS cap
        elapsed = time.time() - start_time
        if elapsed < 1 / fps:
            time.sleep(1 / fps - elapsed)

finally:
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    cv2.destroyAllWindows()
    print("[✔] Drone disarmed, simulation closed.")