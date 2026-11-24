# python3 -m retargeting.bus_probe
import time
from arm import RobotArm

arm = RobotArm('linux').connect()
ph = arm.bus.packetHandler
fingers = arm.fingers
tilt = arm.wrist_tilter

def probe(label):
    time.sleep(0.5)
    out = []
    for sid in (1, 2, 3):
        v, comm, _ = ph.read1ByteTxRx(sid, 62)
        out.append(f"{sid}:{v/10:.1f}V" if comm == 0 else f"{sid}:--")
    print(f"{label:22s} {'  '.join(out)}")

probe("start")
tilt.write(3000, 300, 30);          probe("tilt -> 3000")
tilt.write(2900, 300, 30);          probe("tilt -> 2900")
tilt.write(2800, 300, 30);          probe("tilt -> 2800")
tilt.write(3000, 300, 30);          probe("tilt back")
fingers.set_fingers("release");     probe("fingers release")
fingers.set_fingers("position");    probe("fingers grasp")
fingers.set_fingers("release");     probe("fingers release")