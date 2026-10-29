# Filming version of grip_sim.py: same cost function and minimiser, eased open -> pinch -> hold -> open,
# slow camera orbit. --real drives Hania's fingers in sync.
# Run from the repo root:  python -m retargeting.grip_film [--real] [--needle] [--takes 2]
import argparse
import time
import mediapy as media
import numpy as np
from scipy.optimize import minimize
from arm import ROOT, MjcArm, Calibration, FingerBoard



Q = ['a_index_mcp', 'a_thumb_pip', 'a_thumb_mcp']
# leader joints optimised, in Q order
Q_JOINTS = ['index_mcp_joint', 'thumb_pip_joint', 'thumb_mcp_joint']
FPS = 30
# real PCA9685 channel per leader joint, in Q_JOINTS order (finger_calib.json)
REAL_CH = ['5', '1', '0']     # index, thumb flexion, thumb adduction

# sim radians -> servo us, per leader joint: us = US_AT_ZERO + US_PER_RAD * q
# flexors: q = 0 is straight, top of the MJCF range is full flex (finger_calib straight/flexed)
# adduction: 2.6 deg per 100 us (ratios.json)
US_AT_ZERO = np.array([2200.0, 2000.0, 2200.0])
US_PER_RAD = np.array([(1300 - 2200) / 1.2,          # index_mcp_joint, range top 1.2
                       (800 - 2000) / 1.25,          # thumb_pip_joint, range top 1.25
                       100 / np.radians(2.6)])       # thumb_mcp_joint


def cost(q: list, sim: float, g, pad_i:float, pad_t: float, target_i, target_t, gap):
    sim.set_leader("index_mcp_joint", q[0], g)
    sim.set_leader("thumb_pip_joint", q[1], g)    # thumb flexion
    sim.set_leader("thumb_mcp_joint", q[2], g)    # adduction
    sim.kinematics(g)
    return (np.linalg.norm(g.geom_xpos[pad_i] - g.geom_xpos[pad_t]) - 2 * gap) ** 2


def ease(s):
    return s * s * (3 - 2 * s)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--needle", action="store_true", help="needle between the pads, dropped at the end")
    p.add_argument("--takes", type=int, default=1)
    p.add_argument("--close", type=float, default=6.0)
    p.add_argument("--hold", type=float, default=1.5)
    p.add_argument("--open", type=float, default=2.0)
    p.add_argument("--orbit", type=float, default=40, help="camera sweep over the clip, degrees")
    p.add_argument("--dist", type=float, default=0.3, help="camera distance, m")
    p.add_argument("--real", action="store_true", help="also drive Hania, in sync with the sim")
    p.add_argument("--out", default="grasp_film.mp4")
    a = p.parse_args()

    sim = MjcArm()
    model, mjdata = sim.model, sim.data
    sim.make_renderer()
    joint_on = sim.vis_option(False)
    cam = sim.camera(0.5, 50, -15, [0, 0, 0.5])
    sim.reset()
    g = sim.mj.MjData(model)
    mjdata.joint("forearm_roll").qpos[0] = np.pi
    sim.mj.mj_forward(model, mjdata)

    # index and thumb pads
    pad_i = model.geom("index_pad").id
    pad_t = model.geom("thumb_pad").id
    p_i = mjdata.geom_xpos[pad_i].copy()
    p_t = mjdata.geom_xpos[pad_t].copy()
    centre = (p_i + p_t) / 2
    bounds = [sim.joint_range(j) for j in Q_JOINTS]
    bounds[1] = (bounds[1][0], 0.15)
    a_dir = (p_t - p_i) / np.linalg.norm(p_t - p_i)
    gap = 0.004 + 0.00125
    target_i = centre - a_dir * gap
    target_t = centre + a_dir * gap

    g.qpos[:] = mjdata.qpos
    q = [0.5, 0.0, -0.4]
    result = minimize(cost, q, args=(sim, g, pad_i, pad_t, target_i, target_t, gap), method='L-BFGS-B', bounds=bounds)
    print('main:', result.x, 'left:', result.fun)

    # needle: at the pinch centre, or parked out of shot
    nb = model.body("needle").id
    ng = model.geom("needle_geom").id
    offset = mjdata.geom_xpos[ng] - mjdata.xpos[nb]
    adr = model.jnt_qposadr[model.joint("needle_free").id]
    for j, val in zip(Q_JOINTS, result.x):
        sim.set_leader(j, val, g)
    sim.kinematics(g)
    pinch = (g.geom_xpos[pad_i] + g.geom_xpos[pad_t]) / 2
    mjdata.qpos[adr:adr + 3] = pinch - offset if a.needle else [0, 0, -5]
    sim.mj.mj_forward(model, mjdata)
    cam.lookat, cam.distance = list(pinch), a.dist

    q_open = np.array([mjdata.joint(j).qpos[0] for j in Q_JOINTS])
    q_pinch = result.x
    sim.set_ctrl("a_forearm_roll", np.pi)

    # one schedule for both: per frame (pose 0->1 fraction of open->pinch, drop needle?)
    sched = []

    def phase(f0, f1, seconds, drop=False):
        n = max(1, int(seconds * FPS))
        sched.extend((f0 + ease((f + 1) / n) * (f1 - f0), drop) for f in range(n))

    for t in range(a.takes):
        phase(0, 0, 0.5)
        phase(0, 1, a.close)
        phase(1, 1, a.hold)
        if a.needle and t == a.takes - 1:
            phase(1, 1, 0.4, drop=True)
        phase(1, 0, a.open)
        phase(0, 0, 0.5)

    # real hand: the solved sim angles, converted to us
    us_open = US_AT_ZERO + US_PER_RAD * q_open
    us_pinch = US_AT_ZERO + US_PER_RAD * q_pinch
    print('us open:', us_open.round(), 'us pinch:', us_pinch.round())
    if a.real:
        calib = Calibration.load()
        board = FingerBoard(calib, freq=calib.pwm_freq).connect()
        lo_hi = [board.finger_limits(ch) for ch in REAL_CH]
        try:
            for c in range(3, 0, -1):
                print(c)
                time.sleep(1)
            t0 = time.perf_counter()
            for i, (k, _) in enumerate(sched):
                for ch, us, (lo, hi) in zip(REAL_CH, us_open + k * (us_pinch - us_open), lo_hi):
                    board.move_us(int(ch), int(round(np.clip(us, lo, hi))), verbose=False)
                time.sleep(max(0.0, t0 + (i + 1) / FPS - time.perf_counter()))
        finally:
            board.set_fingers("release", channels=REAL_CH)

    # sim + render from the same schedule
    steps_per_frame = int(round(1 / FPS / model.opt.timestep))
    frames = []
    for i, (k, drop) in enumerate(sched):
        for act, val in zip(Q, q_open + k * (q_pinch - q_open)):
            sim.set_ctrl(act, val)
        if drop:
            mjdata.xfrc_applied[nb] = [0, 0, -0.0005 * 9.81, 0, 0, 0]
        sim.step(steps_per_frame)
        cam.azimuth = 50 - a.orbit / 2 + a.orbit * i / len(sched)
        frames.append(sim.render(cam, joint_on))

    media.write_video(str(ROOT / a.out), frames, fps=FPS)
    print("wrote", ROOT / a.out, len(frames), "frames")


if __name__ == "__main__":
    main()