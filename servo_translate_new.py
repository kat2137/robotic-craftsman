from angle_computation import apply_angles
from Adafruit_PCA9685 import PCA9685
import time
import numpy as np
import sys
import cv2

cap = cv2.VideoCapture("sewing_source.mp4")
if not cap.isOpened():
    print("could not open video")

SERVOS = {
    "pinkie": {"ch": 4, "straight": 1700, "flexed": 1000},
    "thumb_add": {"ch": 0, "straight": 2200, "flexed": 800},
    "middle": {"ch": 2, "straight": 1500, "flexed": 2400},
    "index":  {"ch": 5, "straight": 2400, "flexed": 1300},
    "ring":   {"ch": 3, "straight": 1600, "flexed": 500},
    "thumb":  {"ch": 1, "straight": 2000, "flexed": 800},
}
CHANNELS = [0, 1, 2, 3, 4, 5] 
ORDER = ["thumb", "index", "middle", "ring", "pinkie", "thumb_add"]
# wrist, elbow and shoulder operators are wired to a separate bus servo control board
pwm = PCA9685(busnum=1)
pwm.set_pwm_freq(60)

def move(name, position, angle_min, angle_max):
    s = SERVOS[name]
    frac = float(np.clip((position - angle_min) / (angle_max - angle_min), 0.0, 1.0))
    us = s["straight"] + frac * (s["flexed"] - s["straight"])
    counts = int(np.clip(us / 4.07, 100, 600))
    pwm.set_pwm(s["ch"], 0, counts)

def main():
    data = np.load("handsewing_01.npz")
    joints = data["joints"]
    angles = np.array([apply_angles(i) for i in range(len(joints))])
    lo = np.nanpercentile(angles, 5, axis=0)
    hi = np.nanpercentile(angles, 95, axis=0)

    try:
        last = None
        for row in angles:
            if np.isnan(row).all():
                if last is None:
                    continue
                row = last
            else:
                last = row
            for i, name in enumerate(ORDER):
                move(name, row[i], lo[i], hi[i])
                print(f"{name}: {row[i]:.1f} ({lo[i]:.1f}, {hi[i]:.1f})")
            time.sleep(4/30)
    finally:
        for i, name in enumerate(ORDER):
            move(name, lo[i], lo[i], hi[i])
        time.sleep(0.5)
        for name in ORDER:
            pwm.set_pwm(SERVOS[name]["ch"], 0, 0)
if __name__ == "__main__": main()


