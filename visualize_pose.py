"""Visualize WiLoR hand-pose JSON output (works on macOS, no OpenGL needed).

Usage:
    python visualize_pose.py test1                 # all hands from an image stem
    python visualize_pose.py test1 --out_folder WiLoR/demo_out --img_folder WiLoR/demo_img
    python visualize_pose.py WiLoR/demo_out/test1_hand0.json   # a single json file
"""
import json
import glob
import os
import argparse

import numpy as np
import cv2
import matplotlib
matplotlib.use("Agg")  # headless backend -> save to PNG
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# OpenPose-ordered 21-joint hand skeleton (WiLoR output order)
FINGERS = {
    "thumb":  [0, 1, 2, 3, 4],
    "index":  [0, 5, 6, 7, 8],
    "middle": [0, 9, 10, 11, 12],
    "ring":   [0, 13, 14, 15, 16],
    "pinky":  [0, 17, 18, 19, 20],
}
COLORS = {"thumb": "#e6194b", "index": "#3cb44b", "middle": "#4363d8",
          "ring": "#f58231", "pinky": "#911eb4"}


def load_hands(stem, out_folder):
    if stem.endswith(".json"):
        files = [stem]
    else:
        files = sorted(glob.glob(os.path.join(out_folder, f"{stem}_hand*.json")))
    return [json.load(open(f)) for f in files]


def project_2d(joints, cam_t, focal, W, H, is_right):
    """Project 3D joints to image pixels (pinhole, principal point at centre)."""
    p = joints + np.asarray(cam_t)
    u = focal * p[:, 0] / p[:, 2] + W / 2.0
    v = focal * p[:, 1] / p[:, 2] + H / 2.0
    if not is_right:            # left hands were x-mirrored when saved
        u = W - u
    return np.stack([u, v], axis=1)


def draw_skeleton_2d(ax, uv):
    for finger, idx in FINGERS.items():
        pts = uv[idx]
        ax.plot(pts[:, 0], pts[:, 1], "-", color=COLORS[finger], lw=2)
    ax.scatter(uv[:, 0], uv[:, 1], c="white", edgecolors="black", s=18, zorder=3)


def draw_skeleton_3d(ax, joints, title):
    for finger, idx in FINGERS.items():
        pts = joints[idx]
        ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-o", color=COLORS[finger],
                ms=3, lw=2, label=finger)
    ax.set_title(title, fontsize=9)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    # equal aspect
    lim = np.array([joints.min(0), joints.max(0)])
    center = lim.mean(0); span = (lim[1] - lim[0]).max() / 2
    for setter, c in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), center):
        setter(c - span, c + span)
    ax.view_init(elev=-90, azim=-90)  # look at palm like a camera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", help="image stem (e.g. test1) or a *.json path")
    ap.add_argument("--out_folder", default="WiLoR/demo_out")
    ap.add_argument("--img_folder", default="WiLoR/demo_img")
    ap.add_argument("--save", default=None, help="output PNG path")
    args = ap.parse_args()

    hands = load_hands(args.target, args.out_folder)
    if not hands:
        raise SystemExit(f"No pose JSON found for '{args.target}' in {args.out_folder}")

    stem = os.path.splitext(os.path.basename(hands[0]["image"]))[0]
    img_path = hands[0]["image"]
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) if img is not None else None

    n = len(hands)
    fig = plt.figure(figsize=(4 * (n + 1), 4.5))

    # Panel 1: original image + all hands' 2D skeletons overlaid
    ax0 = fig.add_subplot(1, n + 1, 1)
    if img is not None:
        H, W = img.shape[:2]
        ax0.imshow(img)
        for h in hands:
            uv = project_2d(np.array(h["hand_joints"]), h["camera_translation"],
                            h["focal_length"], W, H, h["is_right_hand"])
            draw_skeleton_2d(ax0, uv)
    ax0.set_title(f"{stem}  ({n} hand{'s' if n > 1 else ''})", fontsize=10)
    ax0.axis("off")

    # Panels 2..: one 3D skeleton per hand
    for i, h in enumerate(hands):
        ax = fig.add_subplot(1, n + 1, i + 2, projection="3d")
        side = "R" if h["is_right_hand"] else "L"
        draw_skeleton_3d(ax, np.array(h["hand_joints"]), f"hand {i} ({side})")

    fig.tight_layout()
    out = args.save or os.path.join(args.out_folder, f"{stem}_viz.png")
    fig.savefig(out, dpi=110)
    print(f"Saved {out}")


if __name__ == "__main__":
    main()
