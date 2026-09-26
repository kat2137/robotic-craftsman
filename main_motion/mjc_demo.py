import mujoco
import mediapy as media
import numpy as np
from f_kinematics import fkine_all, LINKS, BY_NAME
from mj_map import MJ_MAP
XML = '/Users/katarzynadlugosz/projects/robotic-craftsman/model/palm_with_frame.xml'
#Links mapping over to MuJoCo names

model = mujoco.MjModel.from_xml_path(XML)
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)

duration = 3
framerate = 60
# gravity = model.opt.gravity

# joint visualisation
joint_on = mujoco.MjvOption()
joint_on.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

print('positions', data.qpos)
print('velocities', data.qvel)

cam = mujoco.MjvCamera()
cam.distance, cam.azimuth, cam.elevation, cam.lookat = 0.7, 70, -20, [0, 0, 0.5]

for i in range(model.njnt):
    print(i, model.joint(i).name, model.joint(i).type)
for i in range(model.nu):
    print(i, model.actuator(i).name, model.actuator(i).ctrlrange)


def sim_move(cycles, duration):
    frames = []
    mujoco.mj_resetData(model, data)
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
                data.ctrl[model.actuator(entry["act"]).id] = (entry['high']+entry['low'])/2 + (entry['high'] - entry['low'])/2 * np.sin(2*np.pi*cycles*data.time/ duration)
        mujoco.mj_step(model, data)
        if len(frames) < data.time * framerate:
            renderer.update_scene(data, cam, joint_on)
            pixels = renderer.render()
            frames.append(pixels)
        print(f"{data.time:.1f}",
        round(float(data.joint("forearm_roll").qpos[0]), 3),
        round(float(data.joint("wrist_tilt").qpos[0]), 3))
    print("time:", data.time, "duration:", duration, "frames:", len(frames))
    media.write_video("out.mp4", frames, fps=framerate)

if __name__ == "__main__":
    sim_move(2, 12)