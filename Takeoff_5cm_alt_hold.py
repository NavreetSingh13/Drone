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
