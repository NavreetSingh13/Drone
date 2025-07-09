from pymavlink import mavutil
import time
from typing import Any

master: Any = None
target_system: int = 1
target_component: int = 1

def connect_drone(connection_string='/dev/ttyACM0', baudrate=57600):
    global master, target_system, target_component

    print(f"Connecting to MAVLink on {connection_string} at {baudrate} baud...")
    master = mavutil.mavlink_connection(connection_string, baud=baudrate)
    master.wait_heartbeat()
    print("Heartbeat received")

    target_system = master.target_system
    target_component = master.target_component

    mode = 'GUIDED'
    mode_mapping = getattr(master, "mode_mapping", None)
    if callable(mode_mapping):
        mode_id = master.mode_mapping()[mode]
        master.mav.set_mode_send(
            target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print("Mode set to GUIDED")
    else:
        raise RuntimeError("'mode_mapping' not available. May be a log file connection.")

    time.sleep(2)

def arm_and_takeoff(altitude=2.0):
    print("Arming motors...")
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

    while True:
        hb = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if hb and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print("Drone is armed")
            break
        print("Waiting for arming...")
        time.sleep(1)

    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude
    )
    time.sleep(8)

def send_movement_command_YAW(yaw_angle_deg: float = 0.0):
    """
    Rotates the drone by the specified yaw angle in degrees (relative).
    Accepts float input, which is now Pylance-safe.
    """
    print(f"Sending YAW command: {yaw_angle_deg:.2f}°")
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        yaw_angle_deg,
        20,             
        1,              
        1,              
        0, 0, 0
    )
    time.sleep(1)

def send_movement_command_XYA(vx: float, vy: float, altitude: float):
    print(f"Sending XY movement vx={vx:.2f}, vy={vy:.2f}, alt={altitude:.2f}")
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000),
        target_system,
        target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111100011,  # Enable vx, vy, and z
        0, 0, -altitude,
        vx, vy, 0,
        0, 0, 0,
        0, 0
    )

def land():
    print("Landing...")
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(10)

def disarm():
    print("Disarming motors...")
    master.mav.command_long_send(
        target_system,
        target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

def get_battery_info():
    """
    Returns SYS_STATUS (includes battery level, voltage, etc.)
    """
    master.mav.sys_status_send(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
    return msg if msg else "Battery info unavailable"

def get_EKF_status():
    """
    Stub — real EKF monitoring should be done from GCS.
    """
    return "EKF status not available via pymavlink — check GCS"