from angle_computation import apply_angles
from arm import ROOT, NPZ_PATH, FINGER_ORDER, FingerBoard
import time
import numpy as np
import sys

# ch0/ch1/ch5 from finger_calib.json, ch2-4 from arm.EXTRA_FINGERS
# wrist, elbow and shoulder operators are wired to a separate bus servo control board
ORDER = FINGER_ORDER
board = FingerBoard(backend="legacy")
SERVOS = {s.name: {"ch": s.ch, "straight": s.straight, "flexed": s.flexed} for s in board.servos.values()}
CHANNELS = sorted(board.servos)

def move(name, position, angle_min, angle_max):
    board.move_to_angle(name, position, angle_min, angle_max)

def main():
    import cv2
    cap = cv2.VideoCapture(str(ROOT / "sewing_source.mp4"))
    if not cap.isOpened():
        print("could not open video")
    board.connect()

    data = np.load(NPZ_PATH)
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
            board.pwm_off(name)
if __name__ == "__main__": main()
