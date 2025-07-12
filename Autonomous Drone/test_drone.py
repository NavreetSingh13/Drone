import time
from modules import drone

def instructions():
    print("""
========== Drone Manual Control ==========
[r]  Arm and Takeoff (2 meters)
[a]  Yaw Left (-15 deg)
[d]  Yaw Right (+15 deg)
[t]  Takeoff (2 meters)
[l]  Land
[q]  Disarm and Quit
==========================================
""")

def main():
    drone.connect_drone('/dev/ttyACM0', baudrate=115200)
    instructions()

    try:
        while True:
            cmd = input("Enter command [r/a/d/t/l/q]: ").strip().lower()

            if cmd == 'r':
                print("Arming and taking off")
                drone.arm_and_takeoff(2.0)

            elif cmd == 'a':
                print("Yawing left")
                drone.send_movement_command_YAW(-15.0)

            elif cmd == 'd':
                print("Yawing right")
                drone.send_movement_command_YAW(15.0)

            elif cmd == 't':
                print("Taking off to 2 meters")
                drone.arm_and_takeoff(2.0)

            elif cmd == 'l':
                print("Landing...")
                drone.land()

            elif cmd == 'q':
                print("Disarming and exiting")
                drone.disarm()
                break

            else:
                print("Invalid command. Try again.")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Interrupted. Disarming for safety...")
        drone.disarm()

if __name__ == "__main__":
    main()

