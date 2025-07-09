# test_drone.py — Manual drone control using keyboard input

import time
import keyboard  # Requires: pip install keyboard
from modules import drone

def instructions():
    print("""
========== Drone Manual Control ==========
[a]  → Yaw Left  (−15°)
[d]  → Yaw Right (+15°)
[t]  → Takeoff (2 meters)
[l]  → Land
[r]  → Arm
[q]  → Disarm and Quit
==========================================
""")

def main():
    drone.connect_drone('COM', baudrate=115200)
    instructions()

    try:
        while True:
            if keyboard.is_pressed('r'):
                print("→ Arming and taking off")
                drone.arm_and_takeoff(2.0)
                time.sleep(1)

            elif keyboard.is_pressed('a'):
                print("→ Yawing left")
                drone.send_movement_command_YAW(-15.0)
                time.sleep(1)

            elif keyboard.is_pressed('d'):
                print("→ Yawing right")
                drone.send_movement_command_YAW(15.0)
                time.sleep(1)

            elif keyboard.is_pressed('t'):
                print("→ Taking off to 2 meters")
                drone.arm_and_takeoff(2.0)
                time.sleep(1)

            elif keyboard.is_pressed('l'):
                print("→ Landing...")
                drone.land()
                time.sleep(1)

            elif keyboard.is_pressed('q'):
                print("→ Disarming and exiting")
                drone.disarm()
                break

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Interrupted. Disarming for safety...")
        drone.disarm()

if __name__ == "__main__":
    main()