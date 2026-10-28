import numpy as np
import time
from arm import FingerBoard, STServo, RobotArm, EXTRA_FINGERS
q_ref = np.load('refs/clip03.npz')["q_ref"]

arm = RobotArm('linux').connect()
# in what order is clip loaded?
#ref_frames.append([np.radians(elbow_tilt), np.radians(init_roll+roll), np.radians(init_tilt+tilt), (result.x[0]), (result.x[1]), (result.x[2]), (mid_q)])

ORDER = [5, 1, 0, 2]
ST = [(0,3, -1), (1,1,1), (2,2,1)]
RANGES = {5: (-0.05, 1.2), 1: (-0.05, 1.25), 0: (0.1, -1.27), 2: (0.0, 1.6)}   # ch limits in radians
accel =  50  
speed = 100

for row in q_ref:
    if np.isnan(row).any():
        continue
    
    else:
        for i in ST:
            counts = STServo.sim_to_real(i[1], row[i[0]], i[2])
            arm.st(i[1]).write(counts, speed, accel)
        for col, ch in zip(range(3, 7),ORDER):
            straight, flexed = RANGES[ch]
            frac = np.clip((row[col] - straight) / (flexed - straight), 0, 1)
            arm.fingers.move_finger(ch, arm.fingers.servos[ch].frac_to_us(frac))
    time.sleep(1/30)
            


