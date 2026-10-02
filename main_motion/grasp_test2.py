#!/usr/bin/env python3
"""Grasp control for Hania.

Modes:
  no_wrist    - fingers only, fixed calibrated targets
  wrist_grasp - wrist tilt + tenodesis feedforward on the fingers

Talks to the PCA9685 and the ST bus through arm.FingerBoard / arm.STBus.
Keys are read from stdin in cbreak mode, so this works over SSH and does not
need root.
"""

import select
import sys
import termios
import time
import tty

from arm import Calibration, FingerBoard, STBus

# --- calibration -----------------------------------------------------------

calib = Calibration.load()                   # <repo>/finger_calib.json

FINGERS = calib.channels                     # "0" -> {name, position, ...}
CHANNELS = sorted(FINGERS, key=int)          # ["0", "1", "5"]

WRIST_REF = calib.wrist_tilt_ref
WRIST_ID = calib.wrist_id
WRIST_SPEED = calib.wrist_speed
WRIST_ACCEL = calib.wrist_accel
TENODESIS_K = calib.tenodesis_k              # P_f = T_f + k * (P_w - P_w0)

ST_PORT = calib.st_port
ST_BAUD = calib.st_baud
FREQ = calib.pwm_freq

LOOP_DT = 0.02          # key poll interval, s
FINGER_SETTLE_S = 0.3

# --- hardware (opened in main) ---------------------------------------------

board = FingerBoard(calib, freq=FREQ)
bus = STBus()


# --- motion ----------------------------------------------------------------

def validate_calib():
    board.validate(CHANNELS)


def set_fingers(key, wrist_now=None):
    """key is 'position' (grasp) or 'release'. Applies tenodesis if wrist_now given."""
    board.set_fingers(key, wrist_now=wrist_now, channels=CHANNELS)


def move_st(servo_id, position, speed, acceleration):
    bus.servo_by_id(servo_id).move(position, speed, acceleration)


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
    board.connect()
    if not bus.connect(ST_PORT, ST_BAUD):
        quit()

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
        bus.close()


if __name__ == "__main__":
    main()