import cv2
import cvzone
from dronekit import connect, VehicleMode
import time

# Connect to Pixhawk
vehicle = connect('COM6', baud=115200, wait_ready=True)
print(f"Connected to vehicle on: {vehicle}")

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

# Open video capture
cap = cv2.VideoCapture(0)
time.sleep(2)

# Arm and takeoff
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

vehicle.simple_takeoff(5)

while True:
    success, img = cap.read()
    if not success:
        break

    classIds, confs, bbox = net.detect(img, confThreshold=thres, nmsThreshold=nmsThres)
    if len(classIds) != 0:
        for classId, conf, box in zip(classIds.flatten(), confs.flatten(), bbox):
            cvzone.cornerRect(img, box)
            cv2.putText(img, f'{classNames[classId - 1].upper()} {round(conf * 100, 2)}%',
                        (box[0] + 10, box[1] + 30), cv2.FONT_HERSHEY_COMPLEX_SMALL,
                        1, (0, 255, 0), 2)

    cv2.imshow("Drone Camera Feed", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("Landing...")
        vehicle.mode = VehicleMode("LAND")
        break

cv2.destroyAllWindows()
cap.release()
vehicle.close()
