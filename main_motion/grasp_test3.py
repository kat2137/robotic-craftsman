"""
Test 2: is thumb adduction mapped the right way round?

  python3 main_motion/test_thumb_add.py TUCKED OUT
    TUCKED = frame number where the thumb is tucked in towards the index
    OUT    = frame number where the thumb is clearly out

Part A prints the adduction angle for both frames (no hardware).
Part B moves ch0 alone: straight -> flexed -> straight. Watch which way the thumb goes.
"""
import sys
import time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p / ".git").exists())
sys.path.insert(0, str(ROOT))
from angle_computation import apply_angles_arr

NPZ = ROOT / "handsewing_01.npz"
CH, STRAIGHT, FLEXED = 0, 2200, 800        # thumb_add values from SERVOS

tucked, out = int(sys.argv[1]), int(sys.argv[2])

# ---- Part A: data
joints = np.load(NPZ)["joints"]
a_tucked = apply_angles_arr(joints[tucked])[5]
a_out = apply_angles_arr(joints[out])[5]
print(f"tucked frame {tucked}: {a_tucked:.1f} deg")
print(f"out    frame {out}: {a_out:.1f} deg")
if np.isnan(a_tucked) or np.isnan(a_out):
    print("-> one frame has no hand detected, pick another")
elif a_tucked > a_out:
    print("-> data OK: tucked = higher angle = maps to FLEXED")
else:
    print("-> data INVERTED: tucked = lower angle, swap straight/flexed for thumb_add")

# ---- Part B: hardware
input("\nPart B moves ch0 only. Hand near the power switch, press Enter...")
from Adafruit_PCA9685 import PCA9685
pwm = PCA9685(busnum=7)
pwm.set_pwm_freq(60)

def send(us):
    pwm.set_pwm(CH, 0, int(us / 4.07))

try:
    for label, us in (("straight", STRAIGHT), ("flexed", FLEXED), ("straight", STRAIGHT)):
        print(f"ch{CH} -> {label} ({us} us)")
        send(us)
        time.sleep(2)
    print("Did 'flexed' move the thumb TOWARDS the index? If not, the servo side is inverted.")
finally:
    pwm.set_pwm(CH, 0, 0)