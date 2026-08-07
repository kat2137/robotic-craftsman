#!/usr/bin/env python3
"""Grasp control for Hania.

Modes:
  no_wrist    - fingers only, fixed calibrated targets
  wrist_grasp - wrist tilt + tenodesis feedforward on the fingers

Self-contained: talks to the PCA9685 and the ST bus directly, so it does not
import the tuning scripts. Keys are read from stdin in cbreak mode, so this
works over SSH and does not need root.
"""

import json
import os
import select
import sys
import termios
import time
import tty

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

sys.path.append("..")
from scservo_sdk import *

# --- calibration -----------------------------------------------------------

HERE = os.path.dirname(os.path.abspath(__file__))


def _find_calib(name="finger_calib.json", max_up=4):
    d = HERE
    for _ in range(max_up):
        p = os.path.join(d, name)
        if os.path.isfile(p):
            return p
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    raise FileNotFoundError(f"{name} not found at or above {HERE}")


with open(_find_calib()) as f:
    calib = json.load(f)

FINGERS = calib["channels"]                  # "0" -> {name, position, ...}
CHANNELS = sorted(FINGERS, key=int)          # ["0", "1", "5"]

WRIST_REF = calib["wrist_tilt_ref"]
WRIST_ID = calib.get("wrist_id", 2)
WRIST_SPEED = calib.get("wrist_speed", 100)
WRIST_ACCEL = calib.get("wrist_accel", 100)
TENODESIS_K = calib.get("tenodesis_k", 0.02)   # P_f = T_f + k * (P_w - P_w0)

ST_PORT = calib.get("st_port", "/dev/ttyACM0")
ST_BAUD = calib.get("st_baud", 1000000)
FREQ = calib.get("pwm_freq", 60)

LOOP_DT = 0.02          # key poll interval, s
FINGER_SETTLE_S = 0.3

# --- hardware --------------------------------------------------------------

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = FREQ

portHandler = PortHandler(ST_PORT)
packetHandler = sms_sts(portHandler)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

if portHandler.setBaudRate(ST_BAUD):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()


def us_to_duty(us):
    period_us = 1_000_000 / FREQ
    return int(us / period_us * 65535)


def move_us(ch, us):
    print(f"ch {ch}: {us} us")
    pca.channels[ch].duty_cycle = us_to_duty(us)


def read(SCS_ID):
    """Block until the servo stops moving."""
    while 1:
        pos, speed, result, error = packetHandler.ReadPosSpeed(SCS_ID)
        if result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(result))
        else:
            print("[ID:%03d] PresPos:%d PresSpd:%d" % (SCS_ID, pos, speed))
        if error != 0:
            print(packetHandler.getRxPacketError(error))

        moving, result, error = packetHandler.ReadMoving(SCS_ID)
        if result != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(result))

        if moving == 0:
            break
    return


def move_st(servo_id, position, speed, acceleration):
    result, error = packetHandler.WritePosEx(servo_id, position, speed, acceleration)
    if result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(result))
    elif error != 0:
        print("%s" % packetHandler.getRxPacketError(error))
    read(servo_id)


# --- motion ----------------------------------------------------------------

def finger_limits(ch):
    """Return (lo, hi) travel limits in us, ordered low-to-high."""
    cfg = FINGERS[str(ch)]
    return min(cfg["straight"], cfg["flexed"]), max(cfg["straight"], cfg["flexed"])


def validate_calib():
    ok = True
    for ch in CHANNELS:
        cfg = FINGERS[ch]
        lo, hi = finger_limits(ch)
        for key in ("position", "release"):
            if not lo <= cfg[key] <= hi:
                print(f"  ! {cfg['name']}: {key}={cfg[key]} is outside travel {lo}-{hi}")
                ok = False
    if ok:
        print("  calibration OK")


def move_finger(ch, target_us):
    lo, hi = finger_limits(ch)
    pos = int(round(max(lo, min(hi, target_us))))
    if abs(pos - target_us) > 1:
        print(f"  clamped {FINGERS[str(ch)]['name']}: {target_us:.0f} -> {pos}")
    move_us(int(ch), pos)


def set_fingers(key, wrist_now=None):
    """key is 'position' (grasp) or 'release'. Applies tenodesis if wrist_now given."""
    for ch in CHANNELS:
        target = FINGERS[ch][key]
        if wrist_now is not None:
            target += TENODESIS_K * (wrist_now - WRIST_REF)
        move_finger(ch, target)


def wrist_to(pos):
    move_st(WRIST_ID, int(pos), WRIST_SPEED, WRIST_ACCEL)
    return int(pos)


# --- keyboard --------------------------------------------------------------

class KeyReader:
    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("stdin is not a TTY - run this from a terminal")
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get(self, timeout=LOOP_DT):
        """One keypress, or None on timeout. Edge-triggered, no key repeat."""
        if select.select([sys.stdin], [], [], timeout)[0]:
            return sys.stdin.read(1).lower()
        return None

    def prompt(self, text):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)
        try:
            return input(text)
        finally:
            tty.setcbreak(self.fd)


# --- modes -----------------------------------------------------------------

def no_wrist(kb):
    print("[no wrist]  space=grasp  r=release  q=back")
    while True:
        k = kb.get()
        if k == " ":
            print(" grasp")
            set_fingers("position")
            time.sleep(FINGER_SETTLE_S)
        elif k == "r":
            print(" release")
            set_fingers("release")
            time.sleep(FINGER_SETTLE_S)
        elif k == "q":
            return


def wrist_grasp(kb, wrist_target):
    print(f"[wrist grasp]  c=set wrist ({wrist_target})  space=grasp  r=release  q=back")
    while True:
        k = kb.get()
        if k == "c":
            raw = kb.prompt("wrist position: ").strip()
            try:
                wrist_target = int(raw)
            except ValueError:
                print(f" not a number: {raw!r}")
            else:
                print(f" wrist target = {wrist_target}")
        elif k == " ":
            print(" grasp")
            wrist_now = wrist_to(wrist_target)
            set_fingers("position", wrist_now=wrist_now)
            time.sleep(FINGER_SETTLE_S)
        elif k == "r":
            print(" release")
            wrist_to(WRIST_REF)
            set_fingers("release")
            time.sleep(FINGER_SETTLE_S)
        elif k == "q":
            return wrist_target


# --- main ------------------------------------------------------------------

def main():
    print(f"loaded {len(CHANNELS)} channels, wrist id = {WRIST_ID}, ref = {WRIST_REF}")
    validate_calib()

    wrist_target = WRIST_REF
    try:
        with KeyReader() as kb:
            while True:
                print("\nw=wrist grasp  n=no wrist  q=quit")
                k = None
                while k is None:
                    k = kb.get()
                if k == "w":
                    wrist_target = wrist_grasp(kb, wrist_target)
                elif k == "n":
                    no_wrist(kb)
                elif k == "q":
                    return
    finally:
        print("\nreturning to safe pose...")
        try:
            set_fingers("release")
            move_st(WRIST_ID, WRIST_REF, WRIST_SPEED, WRIST_ACCEL)
        except Exception as e:
            print(f"  safe-pose failed: {e}")
        portHandler.closePort()


if __name__ == "__main__":
    main()