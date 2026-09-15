import mujoco
import mediapy as media
import numpy as np
from f_kinematics import fkine_all, LINKS, BY_NAME

model = mujoco.MjModel.from_xml_path('palm_with_frame.xml')
data = mujoco.MjData(model)
renderer = mujoco.Renderer(model)

duration = 3
framerate = 60
frames = []
# gravity = model.opt.gravity

# LINKS name -> MuJoCo. Order matches LINKS exactly.
MJ_MAP = [
    {"link": "base",         "body": "base1",               "joint": None,               "act": None,             "driven": False},
    {"link": "elbow",        "body": "elbow_main1",         "joint": "elbow_pitch",      "act": "a_elbow_pitch",  "driven": False},
    {"link": "wrist rotate", "body": "top_rotation-wheel",  "joint": "forearm_roll",     "act": "a_forearm_roll", "driven": False},
    {"link": "wrist tilt",   "body": "wrist",               "joint": "wrist_tilt",       "act": "a_wrist_tilt",   "driven": False},

    {"link": "index mcp",    "body": "index_pip",           "joint": "index_mcp_joint",  "act": "a_index_mcp",    "driven": False},
    {"link": "index ip",     "body": "index_mip",           "joint": "index_pip_joint",  "act": None,             "driven": True},
    {"link": "index dip",    "body": "index_tip",           "joint": "index_mip_joint",  "act": None,             "driven": True},
    {"link": "index tip",    "body": None,                  "joint": None,               "act": None,             "driven": False},

    {"link": "thumb add",    "body": "thumb_mcp",           "joint": "thumb_mcp_joint",  "act": "a_thumb_mcp",    "driven": False},
    {"link": "thumb mcp",    "body": "thumb_pip",           "joint": "thumb_pip_joint",  "act": "a_thumb_pip",    "driven": False},
    {"link": "thumb ip",     "body": "thumb_tip",           "joint": "thumb_tip_joint",  "act": None,             "driven": True},
    {"link": "thumb tip",    "body": None,                  "joint": None,               "act": None,             "driven": False},
]

# joint visualisation
joint_on = mujoco.MjvOption()
joint_on.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True

print('positions', data.qpos)
print('velocities', data.qvel)

cam = mujoco.MjvCamera()
cam.distance, cam.azimuth, cam.elevation = 0.15, 90, -20

for i in range(model.njnt):
    print(i, model.joint(i).name, model.joint(i).type)
for i in range(model.nu):
    print(i, model.actuator(i).name, model.actuator(i).ctrlrange)

def sim_move(curl_val, cycles, duration):
    mujoco.mj_resetData(model, data)
    for entry in MJ_MAP:
        if entry['act'] is None:
           continue
        else:
            while data.time < duration:
                data.ctrl[model.actuator(entry["act"]).id] = curl_val/2 + curl_val/4 * np.sin(2*np.pi*cycles*data.time/ duration)
                mujoco.mj_step(model, data)
                renderer.update_scene(data, cam, joint)
                pixels = renderer.render()
                frames.append(pixels)

media.show_video(frames, fps=framerate)
sim_move(30, 5, 30)