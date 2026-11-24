from arm import RobotArm, FingerBoard, PinchSensor
import numpy as np
import time

arm = RobotArm('linux')
calib = arm.calib
CHANNELS = list(calib.channels.keys())

WRIST_TILTED = calib.wrist_tilt_ref + 300      # set your tilted position (servo counts)

def move_us(ch, us):
    arm.fingers.move_us(ch, us)

def move_st(servo_id, position, speed, acceleration):
    arm.st(servo_id).move(position, speed, acceleration)

def read(servo_id):
    return arm.st(servo_id).wait_until_stopped()


FIRST = ["index"]            # closes first; try ["thumb"] to compare
GAP = 0.3                    # seconds between the two groups

def grasp(adjust=False):
    shift = 0.0
    if adjust:
        w = read(2)
        if w is not None:
            shift = 0.02 * (w - calib.wrist_tilt_ref)

    def close(finger):
        move_us(int(finger), calib.channels[finger]["position"] + shift)

    first  = [f for f in CHANNELS if any(k in calib.channels[f]["name"].lower() for k in FIRST)]
    second = [f for f in CHANNELS if f not in first]
    for f in first:
        close(f)
    time.sleep(GAP)
    for f in second:
        close(f)

def release():
    for finger in CHANNELS:
        move_us(int(finger), calib.channels[finger]["release"])

def detect(pads):
    return pads.read() or pads.read() or pads.read()

def pinch_cycles(n=10, hold=1.0, rest=1.0):
    pads = PinchSensor(pin=7)
    res = {"normal": [0, 0], "tilted": [0, 0]}          
    try:
        for k in range(n):
            for mode, wrist in (("normal", calib.wrist_tilt_ref), ("tilted", WRIST_TILTED)):
                move_st(2, wrist, 100, 100)
                time.sleep(1.0)
                grasp(adjust=(mode == "tilted"))
                time.sleep(hold)
                closed = detect(pads)
                release()
                time.sleep(rest)
                opened = detect(pads)
                res[mode][0] += closed
                res[mode][1] += opened
                print(f"{k+1}/{n} {mode:6}  closed: {'NEEDLE' if closed else '-'}   open: {'NEEDLE' if opened else '-'}")
    finally:
        release()
        move_st(2, calib.wrist_tilt_ref, 100, 100)
        pads.close()
    for mode, (c, o) in res.items():
        print(f"{mode}: detected {c}/{n}, still detected after release {o}/{n}")

def main():
    arm.connect()
    pinch_cycles(n=10)# 2 to check, then 10

if __name__ == "__main__":
    main()