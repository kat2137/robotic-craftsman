# Filming demo, looped: open -> grasp -> tilt -> roll -> release, repeat until Ctrl+C.
# Run from the repo root:  python3 -m retargeting.grasp_demo
import time
import numpy as np
from arm import RobotArm

DRY_RUN = False          # True = print counts only, nothing moves

# ST bus servo positions in counts
ELBOW = 3200             # inside the usable 2850-3700
ROLL_C = 2000            # wrist roll neutral (ZERO_COUNTS[1])
TILT_C = 3000            # wrist tilt neutral (wrist_tilt_ref = 2999)

ROLL_AMP = 150           # ~13 deg; roll limits allow only counts above 2000
TILT_AMP = 200           # ~18 deg; tilt limits allow only counts below 3000

SPEED, ACCEL = 300, 30
STEPS, STEP_DT = 40, 0.05    # each sweep = 40 steps over 2 s

MIDDLE_OPEN, MIDDLE_CURL = 1500, 1950   # us on ch 2

OPEN_HOLD = 4.0          # s with the hand open (time to place/re-seat the needle)
GRASP_HOLD = 1.0         # s after closing, before the wrist moves
START_DELAY = 5          # s countdown before the first move


def go(servo, counts):
    counts = int(counts)
    print(f"  servo {servo.id}: {counts}")
    if not DRY_RUN:
        servo.write(counts, SPEED, ACCEL)


def sweep(servo, start, end):
    for c in np.linspace(start, end, STEPS):
        go(servo, c)
        time.sleep(STEP_DT)


def hand(key, middle_us):
    print(f"  fingers: {key}")
    if not DRY_RUN:
        fingers.set_fingers(key)
        fingers.move_us(2, middle_us)


arm = RobotArm('linux').connect()
fingers = arm.fingers
fingers.validate()

for s in range(START_DELAY, 0, -1):
    print(f"starting in {s}...")
    time.sleep(1)

# neutral pose, one servo at a time
go(arm.elbow, ELBOW)
time.sleep(1.0)
go(arm.wrist_rotator, ROLL_C)
time.sleep(1.0)
go(arm.wrist_tilter, TILT_C)
time.sleep(1.0)

cycle = 0
try:
    while True:
        cycle += 1
        print(f"\ncycle {cycle}")

        hand("release", MIDDLE_OPEN)
        time.sleep(OPEN_HOLD)

        hand("position", MIDDLE_CURL)
        time.sleep(GRASP_HOLD)

        sweep(arm.wrist_tilter, TILT_C, TILT_C - TILT_AMP)
        time.sleep(0.5)
        sweep(arm.wrist_tilter, TILT_C - TILT_AMP, TILT_C)
        time.sleep(0.5)

        sweep(arm.wrist_rotator, ROLL_C, ROLL_C + ROLL_AMP)
        time.sleep(0.5)
        sweep(arm.wrist_rotator, ROLL_C + ROLL_AMP, ROLL_C)
        time.sleep(0.5)

except KeyboardInterrupt:
    print("\nstopped, returning wrist to neutral")
    go(arm.wrist_tilter, TILT_C)
    go(arm.wrist_rotator, ROLL_C)