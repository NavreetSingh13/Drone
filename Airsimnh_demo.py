import cv2
import cvzone
import airsim
import numpy as np
import time

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
print("Connected to AirSim")

# Set up object detection
thres = 0.55
nmsThres = 0.2

classFile = 'coco.names'
with open(classFile, 'rt') as f:
    classNames = f.read().split('\n')

configPath = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = "frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# Arm and takeoff
client.takeoffAsync().join()
client.moveToZAsync(-5, 3).join()  # Move to 5m above ground

def get_drone_image(client):
    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
    if responses:
        img1d = np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8)
        img_rgba = img1d.reshape(responses[0].height, responses[0].width, 3)
        return img_rgba
    else:
        return None

while True:
    img = get_drone_image(client)
    if img is None:
        continue

    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nmsThres)
    if len(classIds) != 0:
        for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
            cvzone.cornerRect(img, box)
            cv2.putText(img, f'{classNames[classId - 1].upper()} {round(conf * 100, 2)}%',
                        (box[0] + 10, box[1] + 30), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                        1, (0, 255, 0), 2)

    cv2.imshow("AirSim Drone Feed", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Landing...")
        client.landAsync().join()
        client.armDisarm(False)
        client.enableApiControl(False)
        break

cv2.destroyAllWindows()