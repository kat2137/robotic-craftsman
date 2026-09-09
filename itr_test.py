import Jetson.GPIO as GPIO
import time

SENSOR = 7   # BOARD numbering = physical pin 7

GPIO.setmode(GPIO.BOARD)
GPIO.setup(SENSOR, GPIO.IN)

try:
    while True:
        val = GPIO.input(SENSOR)
        print(val, "BLOCKED" if val == GPIO.HIGH else "clear")
        time.sleep(0.15)
except KeyboardInterrupt:
    GPIO.cleanup()