from angle_computation import apply_angles
from Adafruit_PCA9685 import PCA9685
import time
import numpy as np
import sys


CHANNELS = [1, 3, 4, 5, 6] 
#dict format -> "joint_name" : int(channel_number)
SERVO_LIST = {
    "thumb_flex" : 1,
    "thumb_adduction": 2,
    "index": 3,
    "middle": 4,
    "ring": 5,
    "pinkie": 6
}

#dict format -> "value_name" : [angle_val, PWM_val]
SERVO_VAL = {
    "SERVO_MIN": [0, 500],
    "SERVO_MAX": [180, 2500]
}
# wrist, elbow and shoulder operators are wired to a separate bus servo control board

pwm = PCA9685()
pwm.set_pwm_freq(60)

def move(position, channel):
    pulse = int(SERVO_VAL["SERVO_MIN"][1]
            + (position/180) * (SERVO_VAL["SERVO_MAX"][1] - SERVO_VAL["SERVO_MIN"][1]))
    counts = int(pulse / 4.07)  
    print(counts)
    pwm.set_pwm(channel, 0, counts)

def main():
    data = np.load("handsewing_01.npz")    
    valid = ~np.isnan(data["confidence"])
    n_frames = np.flatnonzero(valid)
    for f in range (0, n_frames):
        ang_arr = apply_angles(f)
        for i, angles in enumerate(ang_arr):
            move(angles, CHANNELS[i])
            time.sleep(1/30)

if __name__ == "__main__": main()





