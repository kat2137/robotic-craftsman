
import numpy as np
import sys

path = sys.argv[1]
data = np.load(path)

def bone_vector(p1, p2) -> float:
    p1, p2 = np.array(p1), np.array(p2)
    x1, y1, z1 = p1
    x2, y2, z2 = p2

    vect = [(x1 - x2), (y1 - y2), (z1 - z2)]
    return vect


def finger_angle(mcp: float, pip: float, tip: float) -> float:
    """
    Compute the angle of a finger given its keypoints.
    kp: 3D coordinates of the finger keypoints (mcp, pip, tip)
    mcp: index of the metacarpophalangeal joint
    pip: index of the proximal interphalangeal joint
    tip: index of the fingertip
    Returns the angle in degrees.
    """
    m = bone_vector(mcp, pip)
    t = bone_vector(pip, tip)
    dot_product = np.dot(m,t)
    m_len = np.linalg.norm(m)
    t_len = np.linalg.norm(t)

    cos_theta = dot_product/(m_len * t_len)
    cos_theta = np.clip(cos_theta, -1.0, 1.0)

    angle_radians = np.arccos(cos_theta)
    angle_degrees = np.degrees(angle_radians)
    return angle_degrees

def finger_index (finger:int) -> tuple:
    mcp = 1 + (4 * finger)
    return mcp, mcp + 1, mcp + 3
    
def apply_angles(mcp_row: int) -> list:
    angles = np.full (6, np.nan)
    frame = data["joints"][mcp_row]
    wrist = frame[0]

    #TOD - add thumb adduction
    for f in range (5):
        mcp, pip, tip = finger_index (f)
        total_curl_val = (finger_angle(wrist, frame[mcp], frame[pip])
                          + finger_angle(frame[mcp], frame[pip], frame[tip]))
        angles[f] = total_curl_val
    return angles

