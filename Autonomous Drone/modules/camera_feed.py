camera_feed
import cv2
import subprocess
import time
import os

# Set capture image file path
image_path = "frame.jpg"

# Window setup
cv2.namedWindow("Pi Camera Feed", cv2.WINDOW_AUTOSIZE)

try:
    while True:
        # Capture a frame using libcamera-jpeg silently
        subprocess.run([
            "libcamera-jpeg", 
            "-o", image_path, 
            "--width", "320", 
            "--height", "240", 
            "-n"  # no preview window
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        # Check if image was captured
        if not os.path.exists(image_path):
            print("Failed to capture image.")
            break

        # Read the captured image with OpenCV
        frame = cv2.imread(image_path)

        if frame is None:
            print("Failed to read captured image.")
            break

        # Display the frame
        cv2.imshow("Pi Camera Feed", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    cv2.destroyAllWindows()
    # Remove the last captured image to clean up
    if os.path.exists(image_path):
        os.remove(image_path)