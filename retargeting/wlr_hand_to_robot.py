
import numpy as np
import json
from pathlib import Path

from main_motion.f_kinematics import LINKS

HERE = Path(__file__).resolve().parent      
ROOT = HERE.parent  
OUT = ROOT / "main_motion" / "scale_retarget.json"
 

ORDER = ["thumb_add", "thumb", "middle", "ring", "pinkie", "index"]
FINGERS =  {
        "pinkie": {"ch": 4,},
        "thumb_add": {"ch": 0},
        "middle": {"ch": 2},
        "index":  {"ch": 5},
        "ring":   {"ch": 3},
        "thumb":  {"ch": 1}}

def calculate_distances(data):
    joints = data["joints"]
    real_fingers = []
    for f in range (5):
        mcp, pip, dip, tip = joints[:, (1+4*f)], joints[:, (2+4*f)], joints[:, (3+4*f)], joints[:, (4+4*f)]
        bone1 = pip - mcp
        bone2 = dip - pip
        bone3 = tip - dip
        normal1 = np.linalg.norm(bone1, axis=1)
        normal2 = np.linalg.norm(bone2, axis=1)
        normal3 = np.linalg.norm(bone3, axis=1)
        m, d, t = np.nanmedian(normal1), np.nanmedian(normal2), np.nanmedian(normal3)
        real_fingers.append((m, d, t))
    return real_fingers

def f_lenghts():
    chain_elements = {'index':[], 'thumb':[]}
    for link in LINKS:
            joint = link["name"]
            if joint not in ("base","elbow", "wrist rotate","wrist tilt"):
                t = joint.split(" ")
                if t[0] == 'thumb':
                    if t[1] =='add':
                         continue
                else:
                     if t[1] =='mcp':
                          continue
                chain_elements[t[0]].append((np.asarray(link["offset"], dtype=float)))
                        
    index_lengths = [np.linalg.norm(off) for off in chain_elements["index"]]
    thumb_lenghts = [np.linalg.norm(off) for off in chain_elements["thumb"]]
    return index_lengths, thumb_lenghts
                
def ratios(id_robot, th_robot, fingers):
     ai, bi, ci = fingers[1]
     xi, yi, zi = id_robot
     at, bt, ct = fingers[0]
     xt, yt, zt = th_robot

     i_ratio_mcp, i_ratio_pip, i_ratio_tip = xi/ai, yi/bi, zi/ci
     t_ratio_mcp, t_ratio_pip, t_ratio_tip = xt/at, yt/bt, zt/ct
     return i_ratio_mcp, i_ratio_pip, i_ratio_tip, t_ratio_mcp, t_ratio_pip, t_ratio_tip
    
     
                     
if __name__ == "__main__":
    data = np.load("handsewing_01.npz")
    fingers = calculate_distances(data) 
    index_r, thumb_r = f_lenghts()
    scale1 = sum(index_r)/sum(fingers[1])
    scale2 = sum(thumb_r)/sum(fingers[0])
    print(scale1, scale2)
    with open(OUT, 'w') as f:
         json.dump({'index':(float(scale1), [float(x) for x in index_r]) , 'thumb': (float(scale2), [float(x) for x in thumb_r]), 'clip':"handsewing_01.npz"}, f, indent=4)