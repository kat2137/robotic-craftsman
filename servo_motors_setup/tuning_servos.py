import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

FREQ = 60
CHANNELS = [1, 2, 3, 4, 5, 6]
US_MIN, US_MID, US_MAX = 1000, 1500, 2000

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = FREQ


def us_to_duty(us):
    period_us = 1_000_000 / FREQ
    return int(us / period_us * 65535)

def move_us(ch, us):
    print(f"ch {ch}: {us} us")
    pca.channels[ch].duty_cycle = us_to_duty(us)

ch = int(input("channel: "))
us = 1500
move_us(ch, us)
print("commands: number = go to that us | +N / -N = adjust | c = change channel | q = quit")
while True:
    cmd = input(f"[ch {ch} @ {us} us] > ").strip()
    if cmd == "q":
        break
    elif cmd == "c":
        ch = int(input("channel: "))
    elif cmd.startswith(("+", "-")):
        us += int(cmd)
        move_us(ch, us)
    else:
        try:
            us = int(cmd)
            move_us(ch, us)
        except ValueError:
            print("?")