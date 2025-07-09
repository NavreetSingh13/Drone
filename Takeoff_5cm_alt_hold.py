<<<<<<< HEAD
from pymavlink import mavutil
import time

# Connect to the flight controller
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for heartbeat to confirm connection
master.wait_heartbeat()
print("Heartbeat received from system (system ID: %u)" % master.target_system)

# Set mode to ALT_HOLD
def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    print(f"Mode changed to {mode}")

set_mode("ALT_HOLD")

# Arm the drone
def arm_drone():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("Arming command sent.")

    # Wait until armed
    master.motors_armed_wait()
    print("Drone armed.")

arm_drone()

# Function to read current altitude (from barometer, relative altitude in meters)
def get_altitude():
    msg = master.recv_match(type='VFR_HUD', blocking=True)
    return msg.alt

# Throttle up until 0.05 m (5 cm) altitude
def takeoff_to_5cm():
    print("Starting throttle control for 5 cm ascent")

    target_altitude = 0.05
    throttle_pwm = 1500
    max_pwm = 1600
    min_pwm = 1300

    while True:
        altitude = get_altitude()
        print(f"Current Altitude: {altitude:.2f} m")

        if altitude >= target_altitude:
            print("Reached 5 cm altitude.")
            break

        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            0, 0, throttle_pwm, 0, 0, 0, 0, 0
        )
        time.sleep(0.2)

    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        0, 0, 1500, 0, 0, 0, 0, 0
    )
    print("Holding position.")

takeoff_to_5cm()

time.sleep(5)

def disarm_drone():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0
    )
    print("Disarming command sent.")
    master.motors_disarmed_wait()
    print("Drone disarmed.")

disarm_drone()
print("Mission Complete.")
=======
from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system}, component {master.target_component})")

def set_mode(mode_name):
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    print(f"Mode changed to {mode_name}")

def arm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    print("Arming motors")
    master.motors_armed_wait()
    print("Motors armed")

def disarm():
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
    print("Disarming motors")
    master.motors_disarmed_wait()
    print("Motors disarmed")

set_mode("ALT_HOLD")

arm()

time.sleep(2)

print("Increasing throttle")
master.mav.rc_channels_override_send(
    master.target_system, master.target_component,
    0, 0, 1600, 0, 0, 0, 0, 0)

time.sleep(5)

print("Cutting throttle")
master.mav.rc_channels_override_send(
    master.target_system, master.target_component,
    0, 0, 1000, 0, 0, 0, 0, 0)

disarm()

master.close()
print("Connection closed")
>>>>>>> 54f881ab78082ce20f1e690cf77c94237e257423
