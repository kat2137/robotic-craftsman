#arm kinematics
import math
import numpy as np

from arm import ArmIK, ST_SERVOS

ARM_JOINTS = {
    "wrist_rotate": {"SS_ID": ST_SERVOS["wrist rotate"]["id"]},
    "wrist_tilt": {"SS_ID": ST_SERVOS["wrist tilt"]["id"]},
    "elbow": {"SS_ID": ST_SERVOS["elbow"]["id"]}
}
# link lengths of this planar IK model, mm
UPPER_MM = 324.3
LOWER_MM = 100

# inverse kinematics for estimating wrist position from mcp coordinates
get_joint_pos = ArmIK(UPPER_MM, LOWER_MM).solve
