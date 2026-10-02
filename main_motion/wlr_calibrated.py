import numpy as np
import time

from arm import COUNTS_PER_DEG, FINGER_ORDER, NPZ_PATH, HandPose, RobotArm
from angle_computation import apply_angles_arr

COUNTS = COUNTS_PER_DEG
SPEED = 300
ACCEL = 30
ORDER = FINGER_ORDER

palm_frame = HandPose.palm_frame
tilt = HandPose.tilting
roll = HandPose.rolling

# wrist, elbow and shoulder operators are wired to a separate bus servo control board
arm = RobotArm('linux', finger_backend="legacy")


def step(data):
    move = arm.fingers.move_to_angle
    joints = data["joints"]
    angles = np.array([apply_angles_arr(i) for i in range(len(joints))])
    normal0, xv0, yv0 = palm_frame(joints[0][5], joints[0][17], joints[0][9], joints[0][0])
    init_tilt = arm.st(1).wait()/COUNTS
    init_roll= arm.st(2).wait()/COUNTS
    lo = np.nanpercentile(angles, 5, axis=0)
    hi = np.nanpercentile(angles, 95, axis=0)

    for row in angles:
        try:
            for i, name in enumerate(ORDER):
                move(name, row[i], lo[i], hi[i])
                print(f"{name}: {row[i]:.1f} ({lo[i]:.1f}, {hi[i]:.1f})")
                normal, xv, yv = palm_frame(joints[row][5], joints[row][17], joints[row][9], joints[row][0])
                tilt = tilt(yv0, yv, normal0)
                roll = roll(xv, normal0, normal)

                arm.st(1).move(((init_roll + roll)*COUNTS), SPEED, ACCEL)
                arm.st(2).move(((init_tilt + tilt)*COUNTS), SPEED, ACCEL)
                time.sleep(4/30)
        finally:
            for i, name in enumerate(ORDER):
                move(name, lo[i], lo[i], hi[i])
                time.sleep(0.5)
            for name in ORDER:
                arm.fingers.pwm_off(name)

def main():
    arm.connect()
    data = np.load(NPZ_PATH)


if __name__ == "__main__":
    main()
