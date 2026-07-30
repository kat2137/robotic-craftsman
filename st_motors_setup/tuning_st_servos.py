
import sys
import os
import time

sys.path.append("..")
from scservo_sdk import *                      # Uses FTServo SDK library

ST_SERVOS = {
    "wrist rotate": {"id": 1, "pos": 2058, "straight": 2048, "flexed": 4095},
    "wrist tilt": {"id": 2, "pos": 2043, "straight": 2048, "flexed": 4095},
}

def read(SCS_ID):
    while 1:
        # Read the current position of servo(ID)
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = packetHandler.ReadPosSpeed(SCS_ID)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))
        else:
            print("[ID:%03d] PresPos:%d PresSpd:%d" % (SCS_ID, scs_present_position, scs_present_speed))
        if scs_error != 0:
            print(packetHandler.getRxPacketError(scs_error))

        # Read moving status of servo(ID)
        moving, scs_comm_result, scs_error = packetHandler.ReadMoving(SCS_ID)
        if scs_comm_result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(scs_comm_result))

        if moving==0:
            break
    return
        
# Initialize PortHandler instance and set the port path
portHandler = PortHandler('/dev/ttyACM0')

# Initialize PacketHandler instance, get methods and members of Protocol
packetHandler = sms_sts(portHandler)
    
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baud rate 1000000
if portHandler.setBaudRate(1000000):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

def move_st(servo_id, position, speed, acceleration):
    scs_comm_result, scs_error = packetHandler.WritePosEx(servo_id, position, speed, acceleration)
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    elif scs_error != 0:
        print("%s" % packetHandler.getRxPacketError(scs_error))
    read(servo_id)
   
while True:
    cmd = input(f"servo id:")
    if cmd == "q":
        portHandler.closePort()
        break
    accel = int(input(f"acceleration:"))
    pos = int(input(f"position:"))
    speed = int(input(f"speed:"))
    names = list(ST_SERVOS)
    if cmd in names:
        try:
            move_st(ST_SERVOS[cmd]["id"], pos, speed, accel)
            read(ST_SERVOS[cmd]["id"])
        except ValueError:
            print("?")
    else:
        print (f"Error: servo id {cmd} not found. Valid ids are: {names}")


