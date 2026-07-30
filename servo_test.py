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

try:
    for ch in CHANNELS:
        print(f"\n--- channel {ch} ---")
        move_us(2, US_MID); time.sleep(2)
        move_us(2, US_MIN); time.sleep(2)
        move_us(2, US_MAX); time.sleep(2)
        move_us(2, US_MID); time.sleep(2)
    print("\ndone")
except KeyboardInterrupt:
    for ch in CHANNELS:
        pca.channels[ch].duty_cycle = 0
    print("\nstopped")