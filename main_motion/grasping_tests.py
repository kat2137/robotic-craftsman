from arm import RobotArm, FingerBoard
import numpy as npgi
import json
import time
import keyboard


#value added later
x = 100

arm = RobotArm('linux')
calib = arm.calib
wrist_position = calib.wrist_tilt_ref
CHANNELS = list(calib.channels.keys())
NAMES = [cfg["name"] for ch, cfg in calib.channels.items()]


def move_us(ch, us):
    arm.fingers.move_us(ch, us)

def move_st(servo_id, position, speed, acceleration):
    arm.st(servo_id).move(position, speed, acceleration)

def read(servo_id):
    return arm.st(servo_id).wait()


#move(name, position, angle_min, angle_max):
def no_wrist():
    print ("press 'space' to grasp, 'r' to release, 'q' to quit")
    if keyboard.is_pressed("space"):
        for finger in CHANNELS:
            move_us(int(finger), calib.channels[finger]["position"])
        time.sleep(30)
    if keyboard.is_pressed("r"):
        for finger in CHANNELS:
            move_us(int(finger), calib.channels[finger]["release"])
        time.sleep(30)
    if keyboard.is_pressed("q"):
        exit()

def wrist_grasp():
    print ("press 'ch' to set wrist position, 'space' to grasp, 'r' to release, 'q' to quit")
    if keyboard.is_pressed("ch"):
        wrist_position = int(input("wrist position:"))
    if keyboard.is_pressed("space"):
        move_st(2, wrist_position, 100, 100)
        for finger in CHANNELS:
            ad_pos = calib.channels[finger]["position"] + 0.02 *(read(2) - calib.wrist_tilt_ref)
            move_us(int(finger), ad_pos)
    if keyboard.is_pressed("r"):
        move_st(2, calib.wrist_tilt_ref, 100, 100)
        for finger in CHANNELS:
            move_us(int(finger), calib.channels[finger]["release"])
    if keyboard.is_pressed("q"):
        exit()
        
def main():
    # the unused legacy board is still initialised, as before
    FingerBoard(calib, backend="legacy").connect()
    arm.connect()
    while True:
        print("press 'w' for wrist grasp, 'n' for no wrist, 'q' to quit")
        if keyboard.is_pressed("w"):
            wrist_grasp()
        if keyboard.is_pressed("n"):
            no_wrist()
        if keyboard.is_pressed("q"):
            exit()

if __name__ == "__main__":
    main()
