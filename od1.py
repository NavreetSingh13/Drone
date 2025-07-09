import airsim, cv2, numpy as np, time, math, random, keyboard

# ——— CONNECTION & TAKEOFF ——————————————————————————————
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
print("Connected to AirSim")

client.takeoffAsync().join()
client.moveToZAsync(-5, 2).join()
print("Drone airborne")

# ——— PARAMETERS ——————————————————————————————————————————
fps = 20
focal = 320      # px (assuming 640×480 images)
baseline = 0.5   # m — distance between stereo cameras

speed = 3        # m/s
vz_speed = 2     # vertical speed
yaw_rate = 30    # deg/s

# ——— LOAD DETECTION MODEL ——————————————————————————————
net = cv2.dnn_DetectionModel( # type: ignore
    'frozen_inference_graph.pb',
    'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
)
net.setInputSize(320, 320); net.setInputScale(1/127.5)
net.setInputMean((127.5,)*3); net.setInputSwapRB(True)

# ——— LOOP ———————————————————————————————————————————————
last_time = time.time()

try:
    print("Use W/A/S/D Q/E Space/C to control — ESC to exit.")
    while True:
        start = time.time()
        # — Capture stereo views —
        img_left, img_right = None, None
        res = client.simGetImages([
            airsim.ImageRequest("front_left", airsim.ImageType.Scene, False, False),
            airsim.ImageRequest("front_right", airsim.ImageType.Scene, False, False)
        ])
        if len(res) == 2 and res[0].width != 0:
            img_left = np.frombuffer(res[0].image_data_uint8, np.uint8).reshape(
                res[0].height, res[0].width, 3)
            img_right = np.frombuffer(res[1].image_data_uint8, np.uint8).reshape(
                res[1].height, res[1].width, 3)
            img_left = np.copy(img_left)
            img_right = np.copy(img_right)
        else:
            continue

        # — Key-based velocity —
        vx = vy = vz = yaw = 0
        if keyboard.is_pressed('w'): vx = speed
        if keyboard.is_pressed('s'): vx = -speed
        if keyboard.is_pressed('a'): vy = -speed
        if keyboard.is_pressed('d'): vy = speed
        if keyboard.is_pressed('space'): vz = -vz_speed
        if keyboard.is_pressed('c'): vz = vz_speed
        if keyboard.is_pressed('q'): yaw = -yaw_rate
        if keyboard.is_pressed('e'): yaw = yaw_rate

        client.moveByVelocityAsync(
            vx, vy, vz, 1.0,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
            yaw_mode=airsim.YawMode(True, yaw)
        )

        # — Object detection & stereo calculation —
        classIds, confs, boxes = net.detect(img_left, confThreshold=0.5, nmsThreshold=0.4)
        if isinstance(classIds, np.ndarray) and len(classIds)>0:
            for cid, conf, box in zip(classIds.flatten(), confs.flatten(), boxes):
                x,y,w,h = box
                cx = x + w//2; cy = y + h//2
                cv2.rectangle(img_left, (x,y),(x+w,y+h), (0,255,0), 2)
                cv2.putText(img_left, f"{cid}:{conf:.2f}", (x,y-10),
                            cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)

                # Stereo search along epipolar line (row = cy)
                best_sim, best_disp = -1, None
                ps = 24 # patch size
                patchL = cv2.cvtColor(
                    img_left[max(0,cy-ps//2):cy+ps//2, max(0,cx-ps//2):cx+ps//2],
                    cv2.COLOR_BGR2GRAY
                )
                if patchL.size == 0: continue

                for d in range(1,60):
                    sx = cx - d
                    if sx < 0 or sx+ps > img_right.shape[1]: break
                    patchR = cv2.cvtColor(
                        img_right[max(0,cy-ps//2):cy+ps//2, sx:sx+ps],
                        cv2.COLOR_BGR2GRAY
                    )
                    if patchR.shape != patchL.shape: continue
                    sim = cv2.matchTemplate(patchR, patchL, cv2.TM_CCOEFF_NORMED)[0][0]
                    if sim > best_sim: best_sim, best_disp = sim, d

                if best_disp and best_disp>0:
                    dist = focal*baseline/best_disp
                    # angles from center
                    dx = (cx - img_left.shape[1]/2) / focal
                    dy = (cy - img_left.shape[0]/2) / focal
                    # compute coordinates relative to drone
                    x_rel = dist * dx
                    y_rel = dist * dy
                    z_rel = dist

                    print(f"ID={cid} @ distance={dist:.1f}m"
                          f" | coords=(X:{x_rel:.1f}, Y:{y_rel:.1f}, Z:{z_rel:.1f})")

                    break  # only first detection

        # — Display feeds —
        cv2.imshow("Left View", img_left)
        cv2.imshow("Right View", img_right)
        if cv2.waitKey(1) & 0xFF == 27: break

        # — Frame limiter —
        now = time.time()
        if now - start < 1/fps: time.sleep(1/fps - (now-start))

finally:
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    cv2.destroyAllWindows()
    print("Drone disarmed, session ended.")