from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse

def connectMyCopter():
    parser= argparse.ArgumentParser (description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = args.connect
    baud_rate=57600

    vehicle= connect(connection_string, baud=baud_rate,wait_ready=True)
    return vehicle

def arm():
    while vehicle.is_armable==False:
        print("Waiting for vehicle to become amable..")
        time.sleep(1)
    print("Vehicle is now arnable")
    print("")
    
    vehicle.armed=True
    while vehicle.armed==False:
        print("Waiting for drone to become armed..")
        time.sleep (1)
        
    print("Vehicle is now armed..")
    return None

vehicle=connectMyCopter()
arm()
print("END")