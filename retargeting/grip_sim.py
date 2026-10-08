# minimiser implemented using Aleksandar Haber PhD 'Solve OptimizationProblems in Python Using SciPy minimize()
import mediapy as media
import numpy as np
from scipy.optimize import minimize
from arm import ROOT, NPZ_PATH, Calibration, HandPose, MjcArm, DepthScale, ArmIK, STServo, FingerBoard
from main_motion.f_kinematics import BY_NAME, fkine_all

Q = ['a_index_mcp', 'a_thumb_pip', 'a_thumb_mcp']
# leader joints optimised, in Q order
Q_JOINTS = ['index_mcp_joint', 'thumb_pip_joint', 'thumb_mcp_joint']

palm_frame = HandPose.palm_frame
duration = 6



def cost(q: list, sim: float, g, pad_i:float, pad_t: float, target_i, target_t, gap):
    sim.set_leader("index_mcp_joint", q[0], g)
    sim.set_leader("thumb_pip_joint", q[1], g)    # thumb flexion
    sim.set_leader("thumb_mcp_joint", q[2], g)    # adduction
    sim.kinematics(g)
    return (np.linalg.norm(g.geom_xpos[pad_i] - g.geom_xpos[pad_t]) - 2 * gap) ** 2


def main():
    frames = []
    steps = 0
    
    ik = ArmIK(324.3, 100)
    sim = MjcArm()
    m = sim.model
    i = m.mesh("craft_needle").id
    v = m.mesh_vert[m.mesh_vertadr[i]: m.mesh_vertadr[i] + m.mesh_vertnum[i]]
    print("min:", v.min(axis=0), "max:", v.max(axis=0))
    model, mjdata = sim.model, sim.data
    sim.make_renderer()


    # joint visualisation
    joint_on = sim.vis_option(False)
    cam = sim.camera(0.5, 50, -15, [0, 0, 0.5])
    sim.reset()
    g = sim.mj.MjData(m)
    mjdata.joint("forearm_roll").qpos[0] = np.pi
    sim.mj.mj_forward(model, mjdata)

    # index and thumb pads
    pad_i = model.geom("index_pad").id
    pad_t = model.geom("thumb_pad").id
    p_i = mjdata.geom_xpos[pad_i].copy()
    p_t = mjdata.geom_xpos[pad_t].copy()
    centre = (p_i+p_t)/2
    bounds = [sim.joint_range(j) for j in Q_JOINTS]
    bounds[1] = (bounds[1][0], 0.15)
    a = (p_t - p_i)/np.linalg.norm(p_t - p_i)
    gap = 0.004 + 0.00125 
    target_i = centre - a * gap
    target_t = centre + a * gap

    # locating the needle
    nb = model.body("needle").id
    ng = model.geom("needle_geom").id
    offset = mjdata.geom_xpos[ng] - mjdata.xpos[nb]       # mesh centre vs body origin
    adr = model.jnt_qposadr[model.joint("needle_free").id]
    print(model.dof_damping[model.jnt_dofadr[model.joint("needle_free").id]:][:6])
    print(model.dof_armature[model.jnt_dofadr[model.joint("needle_free").id]:][:6])
    print("target_i:", target_i, " target_t:", target_t)
    sim.kinematics(g)
    g.qpos[:] = mjdata.qpos
    q = [0.5, 0.0, -0.4]  
    result = minimize(cost, q, args=(sim, g, pad_i, pad_t, target_i, target_t, gap), method='L-BFGS-B', bounds=bounds)
    print('main:', result.x, 'left:', result.fun)

    for j, val in zip(Q_JOINTS, result.x):
        sim.set_leader(j, val, g)
    sim.kinematics(g)
    centre = (g.geom_xpos[pad_i] + g.geom_xpos[pad_t]) / 2

    mjdata.qpos[adr:adr + 3] = centre - offset
    sim.mj.mj_forward(model, mjdata)

    i0 = mjdata.joint("index_mcp_joint").qpos[0]
    t0 = mjdata.joint("thumb_mcp_joint").qpos[0]
    a0 = mjdata.joint("thumb_pip_joint").qpos[0]
    n = int(duration / model.opt.timestep)
    traveli = (result.x[0] - i0)/n
    travelt = (result.x[2] - t0)/n
    travela = (result.x[1] - a0)/n
    sim.set_ctrl("a_forearm_roll", np.pi)

    for i in range(int(n)):
        k = i0 +  (i + 1) * traveli
        sim.set_ctrl("a_index_mcp", k)

        t = t0 + (i + 1) * travelt
        sim.set_ctrl("a_thumb_mcp", t)

        a = a0 + (i+1) * travela
        sim.set_ctrl("a_thumb_pip", a)
        sim.step()
        if i % 33 == 0:
            frames.append(sim.render(cam, joint_on))
    mjdata.xfrc_applied[model.body("needle").id] = [0, 0, -0.0005 * 9.81, 0, 0, 0]    
    for i in range(int(2/model.opt.timestep)):
        sim.step()
        frames.append(sim.render(cam, joint_on))
    media.write_video(str(ROOT / "grasp_sim.mp4"), frames, fps=30)



if __name__ == "__main__":
    main()