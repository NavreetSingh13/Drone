from dronekit import connect, VehicleMode

# Connect to the Pixhawk (change 'COM4' to your COM port)
vehicle = connect('COM6', baud=115200, wait_ready=True)

print("Connected to vehicle on: %s" % vehicle)
print("Current Mode: %s" % vehicle.mode.name)

# Close vehicle object before exiting script
vehicle.close()
