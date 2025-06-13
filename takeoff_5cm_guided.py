from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system}, component {master.target_component})")

def set_mode(mode_name):
    mode_id = master.mode_mapping()[mode_name]
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id, 0, 0, 0, 0, 0)
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

def takeoff(altitude):
    print(f"Initiating takeoff to {altitude} meters")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0, altitude)
    time.sleep(1)

def land():
    print("Initiating landing")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0)
    time.sleep(1)

set_mode("GUIDED")

arm()

takeoff(0.05)

print("Holding position for 5 seconds")
time.sleep(5)

land()

master.close()
print("Connection closed")
