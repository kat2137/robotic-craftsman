# sweeps motors and writes the results to a csv for comparison with video

import csv
import sys
import time

sys.path.append("..")

import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from scservo_sdk import *                      # Uses FTServo SDK library

ST_SERVOS = {
    "wrist rotate": {"id": 1, "pos": 2058, "straight": 2048, "flexed": 4095},
    "wrist tilt":   {"id": 2, "pos": 2043, "straight": 2048, "flexed": 4095},
}
ST_SERVOS_L = {
    "wrist rotate": {"id": 1, "min": 2000, "max": 4095},
    "wrist tilt":   {"id": 2, "min": 2200, "max": 3000},
}

FREQ = 60
US_MIN, US_MID, US_MAX = 1000, 1500, 2000

ST_PORT = "/dev/ttyACM0"
ST_BAUD = 1000000
READ_TIMEOUT_S = 3.0
READ_POLL_S = 0.02

LOG_PATH = "position_log.csv"
FIELDNAMES = [
    "step", "element", "channel", "position", "unit",
    "time_cmd", "time_settled", "angle", "note",
]


def blank_row():
    return {k: "" for k in FIELDNAMES}


def sync_marker(write_row):
    """Ties script time to video time. Clap on the prompt."""
    input("clap once in frame, then press Enter: ")
    row = blank_row()
    row.update(step=-1, element="sync", time_cmd=time.time())
    write_row(row)
    print("sync logged")


def sweep_finger(write_row):
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = FREQ

    step = 0

    def us_to_duty(us):
        period_us = 1_000_000 / FREQ
        return int(us / period_us * 65535)

    def move_us(ch, us, step):
        us = max(US_MIN, min(US_MAX, us))          # clamp, don't cook the servo
        t_cmd = time.time()
        pca.channels[ch].duty_cycle = us_to_duty(us)
        row = blank_row()
        row.update(
            step=step, element="finger", channel=ch,
            position=us, unit="us", time_cmd=t_cmd,
        )
        write_row(row)
        print(f"step {step}: ch {ch} -> {us} us")
        return us

    sync_marker(write_row)

    ch = int(input("channel: "))
    us = move_us(ch, US_MID, step)
    step += 1

    print("commands: number = go to that us | +N / -N = adjust | c = change channel | q = quit")
    while True:
        cmd = input(f"[ch {ch} @ {us} us] > ").strip()
        if cmd == "q":
            break
        if cmd == "c":
            ch = int(input("channel: "))
            continue
        try:
            target = us + int(cmd) if cmd.startswith(("+", "-")) else int(cmd)
        except ValueError:
            print("?")
            continue
        us = move_us(ch, target, step)
        step += 1


def sweep_wrist(write_row):
    step = 0

    portHandler = PortHandler(ST_PORT)
    packetHandler = sms_sts(portHandler)

    if not portHandler.openPort():
        print("Failed to open the port")
        return
    if not portHandler.setBaudRate(ST_BAUD):
        print("Failed to change the baudrate")
        portHandler.closePort()
        return

    def wait_until_stopped(servo_id):
        time.sleep(0.15)
        deadline = time.time() + 6.0
        reached = None
        while time.time() < deadline:
            pos, speed, comm, err = packetHandler.ReadPosSpeed(servo_id)
            if comm != COMM_SUCCESS:
                print(packetHandler.getTxRxResult(comm))
            elif err != 0:
                print(packetHandler.getRxPacketError(err))
            else:
                reached = pos

            moving, comm, err = packetHandler.ReadMoving(servo_id)
            if comm != COMM_SUCCESS:
                print(packetHandler.getTxRxResult(comm))
            elif moving == 0:
                return reached

            time.sleep(READ_POLL_S)

        print(f"timeout waiting for servo {servo_id}")
        return reached

    def move_st(name, servo_id, position, speed, accel, step):
        t_cmd = time.time()
        comm, err = packetHandler.WritePosEx(servo_id, position, speed, accel)
        if comm != COMM_SUCCESS:
            print(packetHandler.getTxRxResult(comm))
            return
        if err != 0:
            print(packetHandler.getRxPacketError(err))

        reached = wait_until_stopped(servo_id)
        row = blank_row()
        row.update(
            step=step, element=name, channel=servo_id,
            position=position, unit="counts",
            time_cmd=t_cmd, time_settled=time.time(),
            note="" if reached is None else f"reached {reached}",
        )
        write_row(row)
        print(f"step {step}: {name} -> {position} counts (reached {reached})")

    sync_marker(write_row)

    names = list(ST_SERVOS)
    try:
        while True:
            name = input(f"servo name {names} or q: ").strip()
            if name == "q":
                break
            if name not in ST_SERVOS:
                print(f"Error: {name!r} not found. Valid names are: {names}")
                continue
            try:
                accel = int(input("acceleration: "))
                pos = int(input("position: "))
                speed = int(input("speed: "))
                lo, hi = ST_SERVOS[name]["min"], ST_SERVOS[name]["max"]
                    if not lo <= pos <= hi:
                    print(f"{name}: position must be between {lo} and {hi}")
                    continue
            except ValueError:
                print("?")
                continue
            move_st(name, ST_SERVOS[name]["id"], pos, speed, accel, step)
            step += 1
    finally:
        portHandler.closePort()


def main():
    choice = input("Select calibration - type 'finger' or 'wrist': ").strip().lower()
    if choice not in ("finger", "wrist"):
        print("Invalid input. Please type 'finger' or 'wrist'.")
        return

    # append, so a second run does not wipe the first sweep
    with open(LOG_PATH, "a", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=FIELDNAMES)
        if f.tell() == 0:
            writer.writeheader()

        def write_row(row):
            writer.writerow(row)
            f.flush()                     

        if choice == "finger":
            sweep_finger(write_row)
        else:
            sweep_wrist(write_row)

    print(f"logged to {LOG_PATH}")


if __name__ == "__main__":
    main()