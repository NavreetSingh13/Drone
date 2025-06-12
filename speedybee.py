#sudo apt-get update
# sudo apt-get install python3-pip
# pip3 install pyserial

import serial
import struct
import time

# MSP Command IDs
MSP_SET_RAW_RC = 200
MSP_ARM = 1
MSP_DISARM = 0

# Open serial port to SpeedyBee FC (adjust /dev/serial0 if needed)
ser = serial.Serial('/dev/serial0', 115200, timeout=1)
time.sleep(2)

# Function to build MSP frame
def build_msp_frame(cmd, payload=b''):
    header = b'$M<'
    size = struct.pack('B', len(payload))
    checksum = 0
    for b in payload:
        checksum ^= b
    checksum ^= cmd
    frame = header + size + struct.pack('B', cmd) + payload + struct.pack('B', checksum)
    return frame

# Function to send RC command: [roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4]
def send_rc_command(roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4):
    # 1500 is mid, 1000 min, 2000 max
    payload = struct.pack('<8H', roll, pitch, yaw, throttle, aux1, aux2, aux3, aux4)
    frame = build_msp_frame(MSP_SET_RAW_RC, payload)
    ser.write(frame)

# Arm drone by setting AUX1 to 2000 (adjust based on your Betaflight arm switch config)
def arm_drone():
    print("Arming Drone...")
    for _ in range(20):
        send_rc_command(1500, 1500, 1500, 1000, 2000, 1000, 1000, 1000)
        time.sleep(0.05)

# Throttle up to 1500 (bench test only)
def throttle_up(duration=5):
    print(f"Throttling up for {duration} seconds...")
    start_time = time.time()
    while time.time() - start_time < duration:
        send_rc_command(1500, 1500, 1500, 1500, 2000, 1000, 1000, 1000)
        time.sleep(0.05)

# Disarm by setting AUX1 to 1000
def disarm_drone():
    print("Disarming Drone...")
    for _ in range(20):
        send_rc_command(1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000)
        time.sleep(0.05)

# Main sequence
if __name__ == "__main__":
    try:
        arm_drone()
        throttle_up(5)
        disarm_drone()
        print("Mission Complete.")
    except KeyboardInterrupt:
        disarm_drone()
        print("Interrupted, Drone Disarmed.")
    finally:
        ser.close()
