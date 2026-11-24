import time
from arm import RobotArm, STServo
arm = RobotArm('linux').connect()

TEST = {3: -1, 1: 1, 2: -1}          # servo id: direction as in your ST list
for sid, d in TEST.items():
    for ang in (0.0, 0.3, -0.3, 0.0):
        c = STServo.sim_to_real(sid, ang, d)
        print(f"servo {sid}  angle {ang:+.1f} rad  -> {c} counts")
        arm.st(sid).write(c, 200, 50)
        time.sleep(2.5)
    input("next joint? (Enter)")