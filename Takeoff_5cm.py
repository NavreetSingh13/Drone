from dronekit import connect, VehicleMode
import time

vehicle = connect('/dev/ttyUSB0', baud=57600, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {current_alt:.2f} m")
        if current_alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(0.5)

arm_and_takeoff(0.05)

print("Hold position for 5 seconds")
time.sleep(5)

print("Landing...")
vehicle.mode = VehicleMode("LAND")
vehicle.close()
