import cv2

cams = []

def create_camera(camera_id=0, width=640, height=480):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        raise RuntimeError("Failed to open Pi Camera")
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cams.append(cap)

def get_image_size(camera_id=0):
    cap = cams[camera_id]
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return width, height

def get_video(camera_id=0):
    cap = cams[camera_id]
    ret, frame = cap.read()
    if not ret:
        raise RuntimeError("Failed to read frame from Pi Camera")
    return frame

def close_cameras():
    for cap in cams:
        cap.release()

if __name__ == "__main__":
    create_camera(0)
    while True:
        frame = get_video(0)
        cv2.imshow("Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    close_cameras()
    cv2.destroyAllWindows()