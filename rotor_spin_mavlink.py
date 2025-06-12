#sudo pip3 install pymavlink
from pymavlink import mavutil
import time

# Connect to Pixhawk
print("Connecting to Pixhawk via MAVLink...")
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# Wait for heartbeat
master.wait_heartbeat()
print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

# Set mode to GUIDED (check if your flight stack supports it)
mode = 'GUIDED'
mode_id = master.mode_mapping()[mode]

# Set mode
master.mav.set_mode_send(
    master.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    mode_id
)

# Arm motors
print("Arming motors...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0
)

# Wait for arm confirmation
master.motors_armed_wait()
print("Motors armed ")

# Send manual throttle PWM via RC override
# RC channel map: 1=Roll, 2=Pitch, 3=Throttle, 4=Yaw
print("Spinning rotors for 5 seconds...")

for i in range(50):
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        65535, 65535, 1100, 65535,  # 1100 PWM to throttle (channel 3)
        65535, 65535, 65535, 65535
    )
    time.sleep(0.1)

# Stop throttle
master.mav.rc_channels_override_send(
    master.target_system,
    master.target_component,
    65535, 65535, 1000, 65535,
    65535, 65535, 65535, 65535
)
print("Stopping rotors...")

# Disarm motors
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0
)

# Wait for disarm confirmation
master.motors_disarmed_wait()
print("Motors disarmed ")

print("Test complete. Exiting.")