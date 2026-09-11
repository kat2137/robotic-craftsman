import numpy as np
from f_kinematics import fkine_all
from i_kinematics import get_joint_pos


ELBOW = np.array([128, 0, 0])   # mm; the base offset in LINKS

targets = [[200, 300, 0], [0, 400, 0], [-150, 250, 0], [300, 100, 0]]

for t in targets:
    result = get_joint_pos(t)
    wt, e, mcp, C = result
    pose = fkine_all({"elbow": -e, "wrist tilt": -wt})
    m = pose["index mcp"][:3, 3] * 1000 - ELBOW   # metres -> mm, relative to elbow
    print(f"{t} -> {m.round(1)}  error {np.linalg.norm(mcp - t):.2f} mm")
    print(mcp)