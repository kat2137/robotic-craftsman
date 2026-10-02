
import numpy as np
import sys

from arm import HandPose

bone_vector = HandPose.bone_vector
finger_angle = HandPose.finger_angle
finger_index = HandPose.finger_index
apply_angles_arr = HandPose.apply_angles_arr


def apply_angles(mcp_row: int) -> list:
    """Angles for one row of the module-level `data` (set by the caller, e.g. map_angle.py)."""
    #TOD - add thumb adduction
    return HandPose.apply_angles_arr(data["joints"][mcp_row])

if __name__ == "__main__":
    path = sys.argv[1]
    data = np.load(path)
# for thumb add, the function needs to compute the vector in between the thumb mcp and the index finger mcp.
