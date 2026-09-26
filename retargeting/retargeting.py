# minimiser implemented using Aleksandar Haber PhD 'Solve OptimizationProblems in Python Using SciPy minimize()
import json
import mujoco
import mediapy as media
import numpy as np
import scipy.optimize
from scipy.optimize import minimize
from angle_computation import bone_vector, apply_angles_arr
from main_motion.f_kinematics import BY_NAME, fkine_all
# from main_motion.wlr_calibrated import COUNTS, SPEED, ACCEL
from main_motion.mj_map import MJ_MAP
from pathlib import Path

HERE = Path(__file__).resolve().parent          
ROOT = HERE.parent                              

NPZ    = ROOT / "handsewing_01.npz"
SCALES = ROOT / "main_motion" / "scale_retarget.json"
XML    = ROOT / "model" / "palm_with_frame.xml"

Q = ['index tip', 'thumb tip', 'thumb add']
data = np.load(NPZ)

def palm_frame(mcp_i, mcp_p, mcp_m, w):
    y_axis = np.array(bone_vector(mcp_m, w))
    x_axis = np.array(bone_vector(mcp_i, mcp_p))
    y_axis = y_axis/np.linalg.norm(y_axis)
    x_axis = x_axis/np.linalg.norm(x_axis)
    normal = np.cross(x_axis,y_axis)
    x_axis = np.cross(normal, y_axis)
    return normal, x_axis, y_axis

def tilting(yv0:float, yv:float, normal:float):
    trans_x = np.dot(yv, yv0)
    trans_y = np.dot(normal, yv)
    t = np.degrees(np.arctan2(trans_y, trans_x))
    return t

def rolling(xv:float, normal0:float, normal:float):
    trans_x = np.dot(normal0, xv)
    trans_y = np.dot(normal0, normal)
    t = np.degrees(np.arctan2(trans_y, trans_x))
    return t

def cost(q: list, t1: float, t2: float):
    q = {"index mcp": q[0], "thumb mcp": q[1], "thumb add": q[2]}
    mx = fkine_all(q)
    i_tip = np.linalg.inv(mx["wrist tilt"]) @ mx["index tip"]
    i_tip = i_tip[:3,3]
    t_tip = np.linalg.inv(mx["wrist tilt"]) @ mx['thumb tip']
    t_tip = t_tip[:3,3]
    return np.linalg.norm(i_tip-t1) + np.linalg.norm(t_tip-t2)

def lookup(act):
    if act in Q:
        return Q.index(act)


model = mujoco.MjModel.from_xml_path(str(XML))
mjdata = mujoco.MjData(model)
renderer = mujoco.Renderer(model)

duration = 3
framerate = 60
# gravity = model.opt.gravity

# joint visualisation
joint_on = mujoco.MjvOption()
joint_on.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False

cam = mujoco.MjvCamera()
cam.distance, cam.azimuth, cam.elevation, cam.lookat = 0.7, 70, -20, [0, 0, 0.5]
mujoco.mj_resetData(model, mjdata)
mjdata.joint("forearm_roll").qpos[0] = np.pi

joints = data["joints"]
with open(SCALES) as f: 
    ratios = json.load(f) # robot hand scale ratio
ratio_idx, ratio_th = ratios["index"][0], ratios["thumb"][0]

angles = np.array([apply_angles_arr(joints[i]) for i in range(len(joints))])

normal0, xv0, yv0 = palm_frame(joints[0][5], joints[0][17], joints[0][9], joints[0][0])

scipy.optimize.show_options(solver='minimize', method='L-BFGS-B')
results = []
frames = []
steps = 0

for row in range(200):
    if np.isnan(joints[row]).any():
        continue
    else:
        normal, xv, yv = palm_frame(joints[row][5], joints[row][17], joints[row][9], joints[row][0])

        R = np.array([xv, yv, normal])
        tilt = tilting(yv0, yv, normal0)
        roll = rolling(xv, normal0, normal)

        idx_t, th_t = joints[row][8] - joints[row][5], joints[row][4] - joints[row][1] #the real hand
        idx_t, th_t = R @ idx_t, R @ th_t
        idx_t, th_t = idx_t*ratio_idx, th_t*ratio_th

        target_i = idx_t + BY_NAME["index mcp"]["offset"]
        target_t = th_t + BY_NAME["thumb add"]["offset"]
        # initial guess
    
        q = np.array([np.radians(angles[row][1]), np.radians(angles[row][0]), np.radians(angles[row][5])])
        # solver
        result = minimize(cost, q, args=(target_i, target_t), method='L-BFGS-B', bounds=[(-0.07, 1.2), (-0.07, 1.25), (-0.8, 0.8)])
        results.append([result.fun, result.x])
        print(result.fun, result.x)
        x = results [-1][1]
        q = {'index mcp': x[0], "thumb mcp": x[1], "thumb add": x[2]}
        pose = fkine_all(q)
        steps +=1
        for entry in MJ_MAP:
            if entry['act'] is None:
                continue
            else:
                n = lookup(entry)
                mjdata.ctrl[model.actuator(entry['act']).id] = (entry(results[-1][n]))
        for _ in range(int((1/30) / model.opt.timestep)):
            mujoco.mj_step(model, mjdata)
        
            mjdata.site("index_tip").xpos
            mjdata.body("wrist").xpos

            renderer.update_scene(mjdata, cam, joint_on)
            pixels = renderer.render()
            frames.append(pixels)
vector_angles = np.savez(results)
media.write_video("real_sim.mp4", frames, fps=30)

    


   
         

