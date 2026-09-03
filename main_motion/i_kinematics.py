#arm kinematics
import math
import numpy as np

ARM_JOINTS = {
    "wrist_rotate": {"SS_ID": 1},
    "wrist_tilt": {"SS_ID": 2},
    "elbow": {"SS_ID": 3}
}
# inverse kinematics for estimating wrist position from mcp coordinates
def get_joint_pos(mcp):
    mcp = np.array(mcp)
    xm, ym, zm = mcp
    vect_AD = np.linalg.norm(mcp)
    rxy = np.hypot(xm, ym)

    cos_psi = (vect_AD**2 + 324.3**2 - 100**2) / (2 * 324.3 * vect_AD)
    psi = np.arccos(cos_psi)
    if abs(cos_psi) > 1:
        return None 
    phi = np.arctan2(xm, ym)
    elbow_tilt = np.degrees(phi - psi)

    cos_f = (324.3**2 + 100**2 - vect_AD**2)/(2*324.3*100)
    if abs(cos_f) > 1:
        return None 
    cos_f = np.arccos(cos_f)
    wrist_tilt_a = 180 - np.degrees(cos_f)

    C = (324.3*np.sin(np.radians(elbow_tilt)), 324.3*np.cos(np.radians(elbow_tilt)), 0)

    return wrist_tilt_a, elbow_tilt, mcp, C

w, e, m, C = get_joint_pos([200, 300, 0])
print(np.linalg.norm(np.array(C) - m))






