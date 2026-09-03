import numpy as np
import math
from roboticstoolbox import ET
LINKS = [
    {"name":"base",         "parent": None,         "axis": None,                  "offset": (0.128, 0, 0)},
    {"name":"elbow",        "parent": "base",       "axis": (0,0,1),               "offset": (0, 0, 0)},
    {"name":"wrist rotate", "parent":"elbow",       "axis": (0,1,0),               "offset": (0, 0.1743, 0)},
    {"name": "wrist tilt",  "parent":"wrist rotate","axis": (0,0,1),               "offset": (0, 0.15, 0)},
    {"name": "index mcp",   "parent": "wrist tilt", "axis": (0,0,1),               "offset": (0, 0.1, 0)},
    {"name": "index ip",    "parent": "index mcp",  "axis": (0,0,1),               "offset": (0, 0.041, 0)},
    {"name": "index dip",   "parent": "index ip",   "axis": (0,0,1),               "offset": (0, 0.027, 0)},
    {"name": "thumb add",   "parent": "wrist tilt",  "axis": (0.522, 0.404, 0.747), "offset": (0.005302, 0.027710, 0.019011)},
    {"name": "thumb mcp",   "parent": "thumb add",  "axis": (-0.958, -0.270, -0.092), "offset": (0.010116, 0.039836, 0.022029)},
    {"name": "thumb ip",    "parent": "thumb mcp",  "axis": (-0.927, -0.231, -0.291), "offset": (0.007367, 0.032400, 0.008091)}
]
BY_NAME = {link["name"]: link for link in LINKS}

# rodrigues' rotation formula
def rodrig(joint:str, angle:int):
    K = np.zeros(3)
    joint = BY_NAME[joint]
    axis =joint["axis"]
    K[0][1] = -(axis[2])
    K[0][2]= axis[1]
    K[1][0]= axis[2]
    K[1][2]= -(axis[0])
    K[2][0]= -(axis[1])
    K[2][1]= axis(0)
    R = np.eye(3) + np.sin(angle)*K + (1-np.cos(angle))*K**2

def transform(joint:str, angle:int):
    output = np.eye(4)
    joint = BY_NAME[joint]
    axis =joint["axis"]
    if axis is not None:
        output[:3, :3] = rodrig(joint, angle)
    output[:3, 3] = joint["offset"]
    return output

def find_parent(name):
    link = BY_NAME[name]
    parent = link["parent"]
    if parent is None:
        return [link]
    return find_parent(parent) + [link]

def fkine(joint:int, angle:int):
    T = np.eye(4)
   

    pass

transform("wrist rotate", 30)
rodrig ("elbow", 60)
find_parent("thumb ip")