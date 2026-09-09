import numpy as np
import math
from roboticstoolbox import ET
LINKS = [
    {"name": "base",         "parent": None,          "driver": None,        "ratio": None, "axis": None,                     "offset": (0.128, 0, 0)},
    {"name": "elbow",        "parent": "base",        "driver": None,        "ratio": None, "axis": (0, 0, 1),                "offset": (0, 0, 0)},
    {"name": "wrist rotate", "parent": "elbow",       "driver": None,        "ratio": None, "axis": (0, 1, 0),                "offset": (0, 0.1743, 0)},
    {"name": "wrist tilt",   "parent": "wrist rotate","driver": None,        "ratio": None, "axis": (0, 0, 1),                "offset": (0, 0.15, 0)},

    {"name": "index mcp",    "parent": "wrist tilt",  "driver": None,        "ratio": None, "axis": (0, 0, 1),                "offset": (0, 0.1, 0)},
    {"name": "index ip",     "parent": "index mcp",   "driver": "index mcp", "ratio": 1.0,  "axis": (0, 0, 1),                "offset": (0, 0.041, 0)},
    {"name": "index dip",    "parent": "index ip",    "driver": "index mcp", "ratio": 1.0,  "axis": (0, 0, 1),                "offset": (0, 0.027, 0)},
    {"name": "index tip",    "parent": "index dip",   "driver": None,        "ratio": None, "axis": None,                     "offset": (0, 0.023, 0)},

    {"name": "thumb add",    "parent": "wrist tilt",  "driver": None,        "ratio": None, "axis": (0.522, 0.404, 0.747),    "offset": (0.005302, 0.027710, 0.019011)},
    {"name": "thumb mcp",    "parent": "thumb add",   "driver": None,        "ratio": None, "axis": (-0.958, -0.270, -0.092), "offset": (0.010116, 0.039836, 0.022029)},
    {"name": "thumb ip",     "parent": "thumb mcp",   "driver": "thumb mcp", "ratio": 1.0,  "axis": (-0.927, -0.231, -0.291), "offset": (0.007367, 0.032400, 0.008091)},
    {"name": "thumb tip",    "parent": "thumb ip",    "driver": None,        "ratio": None, "axis": None,                     "offset": (0, 0, 0)},  # TODO: measure
]
BY_NAME = {link["name"]: link for link in LINKS}
CURRENT_ANGLE = [
    {"name":"base",         "angle": None},
    {"name":"elbow",        "angle": None},
    {"name":"wrist rotate", "angle": None},
    {"name": "wrist tilt",  "angle": None},
    {"name": "index mcp",   "angle": None},
    {"name": "index ip",    "angle": None},
    {"name": "index dip",   "angle": None},
    {"name": "thumb add",   "angle": None},
    {"name": "thumb mcp",   "angle": None},
    {"name": "thumb ip",    "angle": None}
]
BY_ANGLE = {angle["name"]: angle for angle in CURRENT_ANGLE}
# rodrigues' rotation formula
def rodrig(joint:str, angle:int):
    K = np.zeros((3,3))
    joint = BY_NAME[joint]
    axis = joint["axis"]
    K[0][1] = -(axis[2])
    K[0][2]= axis[1]
    K[1][0]= axis[2]
    K[1][2]= -(axis[0])
    K[2][0]= -(axis[1])
    K[2][1]= axis[0]
    R = np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*K@K
    return R

def transform(joint:str, angle:int):
    output = np.eye(4)
    joint = BY_NAME[joint]
    axis =joint["axis"]
    if axis is not None:
        output[:3, :3] = rodrig(joint["name"], angle)
    output[:3, 3] = joint["offset"]
    return output

def find_parent(name):
    link = BY_NAME[name]
    parent = link["parent"]
    if parent is None:
        return [link]
    return find_parent(parent) + [link]

def fkine(joint:int):
    M = np.eye(4)
    joints = find_parent(joint)
    for j in joints:
        name = j["name"]
        angle = BY_ANGLE[name]["angle"]
        M_joint = (transform(name, angle))
        M = M @ M_joint
    return M

def fkine_all(q:int):
    pose = {}
    for link in LINKS:
        angle = q.get(link["name"], 0.0)
        b = link["parent"]
        joint = link["name"]
        if b is None:
            pose[joint] = transform(joint, angle)
        else:
            if b in pose:
                x = pose[b] @ transform(joint, angle)
                pose[joint] = x
    return pose
