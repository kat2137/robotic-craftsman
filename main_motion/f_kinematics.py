"""Hand-measured link table for the reference forward kinematics.

LINKS is an independent model used to validate model/palm_with_frame.xml, which
is the source of truth for link offsets, joint axes, ranges and coupling ratios.
The FK maths lives in arm.ReferenceHandFK; the functions below keep the old API.
"""
import numpy as np
import math

from arm import ReferenceHandFK

LINKS = [
    {"name": "base",         "parent": None,          "driver": None,        "ratio": None, "axis": None,                     "offset": (0.128, 0, 0)},
    {"name": "elbow",        "parent": "base",        "driver": None,        "ratio": None, "axis": (0, 0, 1),                "offset": (0, 0, 0)},
    {"name": "wrist rotate", "parent": "elbow",       "driver": None,        "ratio": None, "axis": (0, 1, 0),                "offset": (0, 0.1743, 0)},
    {"name": "wrist tilt",   "parent": "wrist rotate","driver": None,        "ratio": None, "axis": (0, 0, 1),                "offset": (0, 0.15, 0)},

    {"name": "index mcp",    "parent": "wrist tilt",  "driver": None,        "ratio": None, "axis": (0, 0, 1),                "offset": (0, 0.1, 0)},
    {"name": "index ip",     "parent": "index mcp",   "driver": "index mcp", "ratio": 1.314, "axis": (0, 0, 1),                "offset": (0, 0.041, 0)},
    {"name": "index dip",    "parent": "index ip",    "driver": "index mcp", "ratio": 0.429, "axis": (0, 0, 1),                "offset": (0, 0.027, 0)},
    {"name": "index tip",    "parent": "index dip",   "driver": None,        "ratio": None, "axis": None,                     "offset": (0, 0.023, 0)},  # TODO: remeasure after changing the tip

    {"name": "thumb add",    "parent": "wrist tilt",  "driver": None,        "ratio": None, "axis": (0.522, 0.404, 0.747),    "offset": (0.005302, 0.027710, 0.019011)},
    {"name": "thumb mcp",    "parent": "thumb add",   "driver": None,        "ratio": None, "axis": (-0.958, -0.270, -0.092), "offset": (0.010116, 0.039836, 0.022029)},
    {"name": "thumb ip",     "parent": "thumb mcp",   "driver": "thumb mcp", "ratio": 1.273,  "axis": (-0.927, -0.231, -0.291), "offset": (0.007367, 0.032400, 0.008091)},
    {"name": "thumb tip",    "parent": "thumb ip",    "driver": None,        "ratio": None, "axis": None,                     "offset": (0.004524, 0.019896, 0.004968)},  # TODO: remeasure after changing the tip
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
_FK = ReferenceHandFK(LINKS)

rodrig = _FK.rodrig
transform = _FK.transform
find_parent = _FK.find_parent
fkine_all = _FK.fkine_all


def fkine(joint):
    return _FK.fkine(joint, {name: BY_ANGLE[name]["angle"] for name in BY_ANGLE})


if __name__ == "__main__":
    print(fkine_all({"index mcp": 0.0}))
