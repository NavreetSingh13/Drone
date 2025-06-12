#sudo pip3 install dronekit pymavlink
from dronekit import connect, VehicleMode
import time

# Connect to Pixhawk (change the serial port if needed)
print("Connecting to Pixhawk...")
vehicle = connect('/dev/ttyUSB0', baud=57600, wait_ready=True)
# or for UART on Pi: /dev/serial0

# Check connection
print("Connected to: ", vehicle.version)

# Set mode to GUIDED
vehicle.mode = VehicleMode("GUIDED")
while not vehicle.mode.name == 'GUIDED':
    print("Waiting for mode change...")
    time.sleep(1)

# Arm the motors
vehicle.armed = True
while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

print("Motors armed")

# Wait a bit
time.sleep(2)

# Spin rotors (by sending small throttle — if allowed by firmware parameters)
# Only possible if your safety switch is disabled or pressed (if fitted)
print("Spinning rotors slowly for 5 seconds...")

vehicle.channels.overrides['3'] = 1100  # channel 3 is throttle, PWM 1000 = min, 1100-2000 = throttle range

time.sleep(5)

# Stop motors
print("Stopping rotors...")
vehicle.channels.overrides['3'] = 1000

# Disarm the motors
vehicle.armed = False
while vehicle.armed:
    print("Waiting for disarming...")
    time.sleep(1)

print("Motors disarmed")

# Close vehicle connection
vehicle.close()
print("Test complete. Connection closed.")