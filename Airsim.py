import sys
import cv2
import cvzone
import airsim
import numpy as np
import time
import keyboard

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
print("Connected to AirSim")

# Set up object detection model
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
client.moveToPositionAsync(0, 0, 0, 5).join()
client.takeoffAsync().join()
client.moveToZAsync(-3, 3).join()

# Capture image function
def get_drone_image(client):
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    if responses and responses[0].width != 0:
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(responses[0].height, responses[0].width, 3)
        return img_rgb.copy()
    else:
        return None

speed = 3
control_enabled = True
move_duration = 10

try:
    while control_enabled:
        start_time = time.time()

        # Get camera feed and detect objects (same as before)
        img = get_drone_image(client)
        if img is not None:
            classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nmsThres)
            if len(classIds) != 0:
                for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
                    cvzone.cornerRect(img, box)
                    cv2.putText(img, f'{classNames[classId - 1].upper()} {round(conf * 100, 2)}%',
                                (box[0] + 10, box[1] + 30), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                                1, (0, 255, 0), 2)
            cv2.imshow("AirSim Drone Feed", img)
            cv2.waitKey(1)

        # Initialize velocities
        vx = vy = vz = yaw_rate = 0

        if keyboard.is_pressed('w'):
            vx = speed
        if keyboard.is_pressed('s'):
            vx = -speed
        if keyboard.is_pressed('a'):
            vy = -speed
        if keyboard.is_pressed('d'):
            vy = speed
        if keyboard.is_pressed('space'):
            vz = -speed
        if keyboard.is_pressed('c'):
            vz = speed
        if keyboard.is_pressed('q'):
            yaw_rate = -30
        if keyboard.is_pressed('e'):
            yaw_rate = 30

        # Send the latest velocity command (without blocking)
        client.moveByVelocityAsync(vx, vy, vz, move_duration,
                                   drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                                   yaw_mode=airsim.YawMode(True, yaw_rate))

        # Print position info
        state = client.getMultirotorState()
        pos = state.kinematics_estimated.position
        print(f"Pos: x={pos.x_val:.2f}, y={pos.y_val:.2f}, z={pos.z_val:.2f} | Loop: {time.time()-start_time:.3f}s", end='\r')

        if keyboard.is_pressed('esc'):
            print("\nLanding...")
            client.landAsync().join()
            client.armDisarm(False)
            client.enableApiControl(False)
            control_enabled = False
        time.sleep(0.01)

finally:
    cv2.destroyAllWindows()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("\n Drone disarmed and API control released.")
