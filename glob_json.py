import numpy as np
from pathlib import Path
import json

def frame_num (p:Path):
    return int(p.stem.removeprefix("frame").split("_")[0])

HERE = Path(__file__).resolve().parent


def main():
    files = sorted((HERE / "out_demo").glob("frame*.json"), key=frame_num)
    # numbers the frames and puts them into a dict
    frames = [frame_num(p) for p in files]
    #by_num = dict(zip(frames, files))
    n = max(frames) + 1
    joints = np.full((n, 21, 3), np.nan)
    conf = np.full((n), np.nan)
    cam = np.full((n,3), np.nan)
    foc = np.full((n), np.nan)


    for p in files:
        data = json.loads(p.read_text())
        if not data["is_right_hand"]:
            continue
        i = frame_num(p)
        cam[i] = data["camera_translation"]
        joints[i] = data["hand_joints"]
        foc[i] = data['focal_length']
        if "confidence" in data:
            conf[i] = data["confidence"]

    np.savez(HERE / "handsewing_01.npz", joints=joints, camera_translation=cam, confidence=conf, focal_length=foc)

if __name__ == "__main__":
    main()
