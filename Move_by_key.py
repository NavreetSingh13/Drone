import cv2
import numpy as np
import time
import keyboard
import airsim
# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)
print("connectected to 2")
# Take off
client.takeoffAsync().join()

print("Use 'WASD' to move, 'Q/E' to move up/down, and 'Esc' to exit.")
def mov(client,a,b,c,d):
    client.moveByVelocityAsync(a, b, c, d).join()
waypoints = [ (50, 20, -1)]  # List of target positions

for wp in waypoints:
    x, y, z = wp
    print(f"Moving to ({x}, {y}, {z})...")
    client.moveToPositionAsync(x, y, z, 5).join()
    time.sleep(3)

print("All waypoints reached!")
while True:
    a=b=c=0
    d=0.01
    # Drone control based on key press
    if keyboard.is_pressed('w'):
        a+=4  # Move forward
    if keyboard.is_pressed('s'):
        a=a-4 # Move backward
    if keyboard.is_pressed('a'):
        b=b-4  # Move left
    if keyboard.is_pressed('d'):
        b=b+4 # Move right
    if keyboard.is_pressed('space'):
        c=c+1  # Take off
    if keyboard.is_pressed('c'):
        c=c-1
    state = client.getMultirotorState()
    position = state.kinematics_estimated.position
    x, y, z = position.x_val, position.y_val, position.z_val
    print(x,y,z)
    mov(client,a,b,c,d)
    time.sleep(0.01)