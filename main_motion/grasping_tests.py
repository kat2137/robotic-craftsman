from servo_motors_setup.tuning_servos import move_us
from st_motors_setup.tuning_st_servos import move_st, read
from angle_computation import apply_angles
from Adafruit_PCA9685 import PCA9685
import numpy as np
import json
import time
import keyboard


#value added later
x = 100

pwm = PCA9685(busnum=1)
pwm.set_pwm_freq(60)

with open("finger_calib.json") as f:
    calib = json.load(f)
wrist_position = calib["wrist_tilt_ref"]
CHANNELS = list(calib["channels"].keys())
NAMES = [cfg["name"] for ch, cfg in calib["channels"].items()]


#move(name, position, angle_min, angle_max):
def no_wrist():
    print ("press 'space' to grasp, 'r' to release, 'q' to quit")
    if keyboard.is_pressed("space"):
        for finger in CHANNELS:
            move_us(int(finger), calib["channels"][finger]["position"])
        time.sleep(30)
    if keyboard.is_pressed("r"):
        for finger in CHANNELS:
            move_us(int(finger), calib["channels"][finger]["release"])
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
            ad_pos = calib["channels"][finger]["position"] + 0.02 *(read(2) - 3000)
            move_us(int(finger), ad_pos)
    if keyboard.is_pressed("r"):
        move_st(2, 3000, 100, 100)
        for finger in CHANNELS:
            move_us(int(finger), calib["channels"][finger]["release"])
    if keyboard.is_pressed("q"):
        exit()
        
def main():
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