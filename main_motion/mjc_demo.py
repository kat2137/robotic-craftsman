import mediapy as media
import numpy as np
from arm import ROOT, MjcArm
from main_motion.f_kinematics import fkine_all, LINKS, BY_NAME
from main_motion.mj_map import MJ_MAP
#Links mapping over to MuJoCo names

duration = 3
framerate = 60
# gravity = model.opt.gravity


def sim_move(sim, cycles, duration):
    model, data = sim.model, sim.data
    # joint visualisation
    joint_on = sim.vis_option(True)
    cam = sim.camera(0.7, 70, -20, [0, 0, 0.5])
    frames = []
    sim.reset()
    data.joint("forearm_roll").qpos[0] = np.pi
    #i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "needle_grip")
    #data.eq_active[i] = 0
    steps = 0
    max_steps = int(duration/model.opt.timestep)+100
    while data.time < duration and steps < max_steps:
        steps += 1
        #print("weld active:", data.eq_active[i])
        for entry in MJ_MAP:
            if entry['act'] is None:
                continue
            else:
                sim.set_ctrl(entry["act"], (entry['high']+entry['low'])/2 + (entry['high'] - entry['low'])/2 * np.sin(2*np.pi*cycles*data.time/ duration))
        sim.step()
        if len(frames) < data.time * framerate:
            frames.append(sim.render(cam, joint_on))
        print(f"{data.time:.1f}",
        round(float(data.joint("forearm_roll").qpos[0]), 3),
        round(float(data.joint("wrist_tilt").qpos[0]), 3))
    print("time:", data.time, "duration:", duration, "frames:", len(frames))
    media.write_video(str(ROOT / "out.mp4"), frames, fps=framerate)


def main():
    sim = MjcArm()
    sim.make_renderer()
    model, data = sim.model, sim.data

    print('positions', data.qpos)
    print('velocities', data.qvel)

    for i in range(model.njnt):
        print(i, model.joint(i).name, model.joint(i).type)
    for i in range(model.nu):
        print(i, model.actuator(i).name, model.actuator(i).ctrlrange)

    sim_move(sim, 2, 12)


if __name__ == "__main__":
    main()
