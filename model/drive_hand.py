"""
base driver script for testing initial model, pre-calibration and measuring ratios. Currently innacurate.
"""

import argparse
import json
import time

import mujoco
import mujoco.viewer
import numpy as np
SERVO_UNITS = "sts3215"          # "sts3215" | "degrees"

COUPLING = {
    "index":  {"a_index_mcp": 1.0, "a_index_pip": -1.0, "a_index_mip": 0.8},
    "middle": {"a_mid_mcp":   1.0, "a_mid_pip":    1.0, "a_mid_mip":   0.8},
    "ring":   {"a_ring_mcp":  1.0, "a_ring_pip":   1.0, "a_ring_mip":  0.8},
    "small":  {"a_small_mcp": 1.0, "a_small_pip":  1.0, "a_small_mip": 0.8},
    "thumb":  {"a_thumb_pip": 1.0, "a_thumb_tip":  0.8},
}
SERVO_OPEN   = 2048
SERVO_CLOSED = 3100

# Joint angle (rad) the finger reaches at SERVO_CLOSED, for a ratio-1.0 joint.
FULL_CURL = 1.5
DIRECT = ["a_elbow_pitch", "a_forearm_roll", "a_wrist_tilt", "a_thumb_mcp"]


def servo_to_frac(v):
    """Servo count (or degrees) -> 0.0 open .. 1.0 closed."""
    if SERVO_UNITS == "degrees":
        lo, hi = SERVO_OPEN * 360.0 / 4096, SERVO_CLOSED * 360.0 / 4096
    else:
        lo, hi = SERVO_OPEN, SERVO_CLOSED
    return float(np.clip((v - lo) / (hi - lo), 0.0, 1.0))


class Hand:
    def __init__(self, xml="palm.xml"):
        self.model = mujoco.MjModel.from_xml_path(xml)
        self.data = mujoco.MjData(self.model)
        self.act = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            self.act[name] = i
        self.ctrl_range = self.model.actuator_ctrlrange.copy()

    def set_actuator(self, name, value):
        i = self.act[name]
        lo, hi = self.ctrl_range[i]
        self.data.ctrl[i] = float(np.clip(value, lo, hi))

    def apply_servo_pose(self, pose):
        """pose: {"index": 2600, "thumb": 2900, "a_wrist_tilt": 0.3, ...}

        Finger names go through COUPLING; anything in DIRECT is taken as
        radians and written straight to that actuator.
        """
        for key, val in pose.items():
            if key in COUPLING:
                frac = servo_to_frac(val)
                for act_name, ratio in COUPLING[key].items():
                    self.set_actuator(act_name, frac * FULL_CURL * ratio)
            elif key in DIRECT:
                self.set_actuator(key, val)
            elif key in self.act:
                self.set_actuator(key, val)
            else:
                raise KeyError(f"unknown pose key: {key}")

    def current_ctrl(self):
        return self.data.ctrl.copy()


def interpolate(hand, target_ctrl, seconds, viewer, settle=0.0):
    """Ramp ctrl from where it is now to target over `seconds`, in real time."""
    start = hand.current_ctrl()
    dt = hand.model.opt.timestep
    n = max(1, int(seconds / dt))
    for k in range(n):
        alpha = (k + 1) / n
        # smoothstep, so the servos don't jerk at the ends
        a = alpha * alpha * (3 - 2 * alpha)
        hand.data.ctrl[:] = start + a * (target_ctrl - start)
        mujoco.mj_step(hand.model, hand.data)
        if viewer is not None:
            viewer.sync()
            time.sleep(dt)
    for _ in range(int(settle / dt)):
        mujoco.mj_step(hand.model, hand.data)
        if viewer is not None:
            viewer.sync()
            time.sleep(dt)


def play(hand, sequence, viewer):
    """sequence: list of {"pose": {...}, "time": 1.5, "hold": 0.5}"""
    for step in sequence:
        hand.apply_servo_pose(step["pose"])
        target = hand.current_ctrl()
        # rewind ctrl so interpolate() can ramp into it
        hand.data.ctrl[:] = hand.current_ctrl()
        interpolate(hand, target, step.get("time", 1.0), viewer,
                    settle=step.get("hold", 0.0))


DEMO = [
    {"pose": {"index": SERVO_OPEN, "middle": SERVO_OPEN, "ring": SERVO_OPEN,
              "small": SERVO_OPEN, "thumb": SERVO_OPEN,
              "a_wrist_tilt": 0.0, "a_elbow_pitch": 0.0},
     "time": 1.0, "hold": 0.5},

    {"pose": {"index": SERVO_CLOSED, "thumb": SERVO_CLOSED,
              "a_thumb_mcp": 0.5},
     "time": 1.5, "hold": 1.0},

    {"pose": {"a_wrist_tilt": 0.6}, "time": 1.0, "hold": 0.5},
    {"pose": {"a_wrist_tilt": -0.3}, "time": 1.0, "hold": 0.5},

    {"pose": {"index": SERVO_OPEN, "thumb": SERVO_OPEN, "a_thumb_mcp": 0.0},
     "time": 1.0, "hold": 1.0},
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xml", default="palm.xml")
    ap.add_argument("--grasp", help="JSON file of servo positions")
    ap.add_argument("--headless", action="store_true")
    args = ap.parse_args()

    hand = Hand(args.xml)

    if args.grasp:
        with open(args.grasp) as f:
            raw = json.load(f)
        sequence = raw if isinstance(raw, list) else [{"pose": raw, "time": 2.0, "hold": 2.0}]
    else:
        sequence = DEMO

    if args.headless:
        play(hand, sequence, None)
        print("done, final qpos:", np.round(hand.data.qpos, 3))
        return

    with mujoco.viewer.launch_passive(hand.model, hand.data) as viewer:
        while viewer.is_running():
            play(hand, sequence, viewer)


if __name__ == "__main__":
    main()
