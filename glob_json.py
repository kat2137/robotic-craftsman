import numpy as np
from pathlib import Path
import json

def frame_num (p:Path):
    return int(p.stem.removeprefix("frame").split("_")[0])

files = sorted(Path("video_out").glob("frame*.json"), key=frame_num)
# numbers the frames and puts them into a dict
frames = [frame_num(p) for p in files]
#by_num = dict(zip(frames, files))
n = max(frames) + 1
joints = np.full((n, 21, 3), np.nan)
conf = np.full((n), np.nan)
hand = np.full((n), np.nan)
camera_translation = np.full((n,3), np.nan)


for p in files:
    data = json.loads(p.read_text())
    if not data["is_right_hand"]:
        continue
    i = frame_num(p)
    joints[i] = data["hand_joints"]
    conf[i] = data["confidence"]

np.savez("handsewing_01.npz", joints=joints, confidence=conf, hand=hand)