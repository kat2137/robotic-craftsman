import numpy as np
import time
from arm import FingerBoard, STServo, RobotArm, EXTRA_FINGERS, PinchSensor
q_ref = np.load('refs/clip05.npz')["q_ref"]

arm = RobotArm('linux').connect()
# in what order is clip loaded?
#ref_frames.append([np.radians(elbow_tilt), np.radians(init_roll+roll), np.radians(init_tilt+tilt), (result.x[0]), (result.x[1]), (result.x[2]), (mid_q)])

ORDER = [5, 1, 0, 2]
ST = [(0,3, -1), (1,1,1), (2,2,-1)]
OFFSET = {3: 0.0, 1: np.pi, 2: 0.0}
RANGES = {5: (-0.05, 1.2), 1: (-0.05, 1.25), 0: (0.1, -1.27), 2: (0.0, 1.6)}   # ch limits in radians
accel =  50  
speed = 600

pads = PinchSensor(pin=7)
pad_log = []

for row in q_ref:
    if np.isnan(row).any():
        continue
    
    else:
        for i in ST:
            if i[1] == 1:
                angle = row[1] % (2 * np.pi) - np.pi      # roll, wrapped
            else:
                angle = row[i[0]]
            counts = STServo.sim_to_real(i[1], angle, i[2])
            print(i[1], counts)
            arm.st(i[1]).write(counts, speed, accel)
            if i[1] == 3:
                print("elbow", round(float(angle), 3), counts)
        for col, ch in zip(range(3, 7),ORDER):
            straight, flexed = RANGES[ch]
            frac = np.clip((row[col] - straight) / (flexed - straight), 0, 1)
            arm.fingers.move_finger(ch, arm.fingers.servos[ch].frac_to_us(frac))
        pad_log.append(pads.read())
        print("NEEDLE" if pad_log[-1] else "-")
    time.sleep(1/30)
pads.close()
np.save("pad_log.npy", np.array(pad_log))


