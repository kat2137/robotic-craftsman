# minimiser implemented using Aleksandar Haber PhD 'Solve OptimizationProblems in Python Using SciPy minimize()
import mediapy as media
import numpy as np
from scipy.optimize import minimize
from arm import ROOT, NPZ_PATH, Calibration, HandPose, MjcArm, DepthScale, ArmIK, STServo, FingerBoard
from main_motion.f_kinematics import BY_NAME, fkine_all

Q = ['a_index_mcp', 'a_thumb_pip', 'a_thumb_mcp']
# leader joints optimised, in Q order
Q_JOINTS = ['index_mcp_joint', 'thumb_pip_joint', 'thumb_mcp_joint']
W = 30

palm_frame = HandPose.palm_frame
cam_translation = DepthScale.cam_tr
tilting = HandPose.tilting
rolling = HandPose.rolling

duration = 3
framerate = 60
ref_frames = []
t1 = 0.012

def cost(q: list, t1: float, t2: float):
    q = {"index mcp": q[0], "thumb mcp": q[1], "thumb add": q[2]}
    mx = fkine_all(q)
    i_tip = np.linalg.inv(mx["wrist tilt"]) @ mx["index tip"]
    i_tip = i_tip[:3,3]
    t_tip = np.linalg.inv(mx["wrist tilt"]) @ mx['thumb tip']
    t_tip = t_tip[:3,3]
    return np.linalg.norm(i_tip-t1) + np.linalg.norm(t_tip-t2)

def grip_detect(d, was_gripping, t_on, t_off):
    if was_gripping:
        return d < t_off      
    return d < t_on           

def touch_solver(sim, g, theta):
    i_d = g.site_xmat[sim.site_id('index_touch_site')].reshape(3, 3)[:, 2]
    t_d = g.site_xmat[sim.site_id("thumb_touch_site")].reshape(3, 3)[:, 2]
    angle = np.arccos(np.clip(-np.dot(i_d, t_d), -1, 1))
    return (angle - theta) ** 2

def get_theta(GRIP_Q, sim, g):
    print("grip gap mm:", pad_gap(GRIP_Q, sim, g) * 1000)
    i_d = g.site_xmat[sim.site_id('index_touch_site')].reshape(3, 3)[:, 2]
    t_d = g.site_xmat[sim.site_id("thumb_touch_site")].reshape(3, 3)[:, 2]
    return np.arccos(np.clip(-np.dot(i_d, t_d), -1, 1))

def pad_gap(q, sim, g):
    for j, val in zip(Q_JOINTS, q):
        sim.set_leader(j, val, g)
    sim.kinematics(g)
    return np.linalg.norm(sim.site_pos("index_touch_site", g) - sim.site_pos("thumb_touch_site", g))

MIN_GAP = 2.0   #needle diameter
def grasp_cost(q, sim, g, theta):
    gap = (pad_gap(q, sim, g) * 1000 - MIN_GAP) ** 2
    return gap + W * touch_solver(sim, g, theta)
    


def main(theta):
    data = np.load(NPZ_PATH)
    sim = MjcArm()
    ik = ArmIK(324.3, 100)
    m = sim.model
    i = m.mesh("craft_needle").id
    v = m.mesh_vert[m.mesh_vertadr[i]: m.mesh_vertadr[i] + m.mesh_vertnum[i]]
    print("min:", v.min(axis=0), "max:", v.max(axis=0))
    model, mjdata = sim.model, sim.data
    sim.make_renderer()
    print(data.files)

    # joint visualisation
    joint_on = sim.vis_option(False)
    cam = sim.camera(0.7, 60, -20, [0, 0, 0.5])
    sim.reset()
    mjdata.xfrc_applied[model.body("needle").id] = [0, 0, 0.0005 * 9.81, 0, 0, 0]
    mjdata.joint("forearm_roll").qpos[0] = np.pi
    sim.mj.mj_forward(model, mjdata)
    pad_i = mjdata.geom_xpos[model.geom("index_pad").id]
    pad_t = mjdata.geom_xpos[model.geom("thumb_pad").id]
    centre = (pad_i+pad_t)/2

    nb = model.body("needle").id
    ng = model.geom("needle_geom").id
    offset = mjdata.geom_xpos[ng] - mjdata.xpos[nb]       # mesh centre vs body origin
    adr = model.jnt_qposadr[model.joint("needle_free").id]
    mjdata.qpos[adr:adr + 3] = centre - offset
    print(model.dof_damping[model.jnt_dofadr[model.joint("needle_free").id]:][:6])
    print(model.dof_armature[model.jnt_dofadr[model.joint("needle_free").id]:][:6])

    joints = data["joints"]
    scales = Calibration.load().hand_scales # robot hand scale ratio
    ratio_idx, ratio_th = scales["index"], scales["thumb"]
    bounds = [sim.joint_range(j) for j in Q_JOINTS]
    grip_bounds = list(bounds)
    grip_bounds[1] = (grip_bounds[1][0], 0.1)   

    angles = np.array([HandPose.apply_angles_arr(joints[i]) for i in range(len(joints))])
    mid = np.radians(angles[:, 2])     # middle finger in radians
    
    results = []
    frames = []
    gripped = []
    rows = []
    steps = 0

    first = np.flatnonzero(~np.isnan(joints).any(axis=(1, 2)))[0]
    normal0, xv0, yv0 = palm_frame(joints[first][5], joints[first][17], joints[first][9], joints[first][0])
    t = DepthScale.cam_tr(data["camera_translation"], np.nanmedian(data["focal_length"]))
    mcp_cam = t + joints[:, 9]
    targets = DepthScale.smooth(DepthScale.ik_targets(mcp_cam))
    print("fl:", np.nanmedian(data["focal_length"][:3]))
    print("raw:", data["camera_translation"][:3])
    print("corrected:", t[:3])
    d = np.linalg.norm(targets, axis=1)
    print("target distance mm:", np.nanmin(d), np.nanmax(d))
    print("first targets:", targets[:3])

    prev_tilt, prev_roll = None, None
    a = 0.3  #EMA
    normal, xv, yv = palm_frame(joints[first][5], joints[first][17], joints[first][9], joints[first][0])
    init_tilt = tilting(yv0, yv, normal0)
    init_roll = rolling(xv, normal0, normal)

    gripping = False
    tip_d = np.linalg.norm(joints[:, 8] - joints[:, 4], axis=1)
    win = np.lib.stride_tricks.sliding_window_view(np.pad(tip_d, 2, mode='edge'), 5)
    tip_d = np.nanmedian(win, axis=1)        # 5-frame median, NaN-safe
    for row in range(len(joints)):   
        
        if np.isnan(joints[row]).any():
            continue
        else:
            print("target:", targets[row])
            result = ik.solve(targets[row])   
            elbow_tilt = 0.0       # ← here
            if result is not None:
                elbow_tilt = result[1]
            normal, xv, yv = palm_frame(joints[row][5], joints[row][17], joints[row][9], joints[row][0])

            gripping = grip_detect(tip_d[row], gripping, t1, 1.5 * t1)
            if gripping == True:
                gripped.append(gripping)
                rows.append(row)
                print(f'gripping',{row})
            print(elbow_tilt)
            R = np.array([-normal, yv, xv])
            tilt = tilting(yv0, yv, normal0)
            roll = rolling(xv, normal0, normal)
            if prev_tilt is None:
                prev_tilt, prev_roll = tilt, roll

            delta_tilt = (tilt - prev_tilt + 180) % 360 - 180  #ema for wrist tilt
            delta_roll = (roll - prev_roll + 180) % 360 - 180 #ema for wrist roll
            tilt = prev_tilt + a * delta_tilt
            roll = prev_roll + a * delta_roll

            prev_tilt, prev_roll = tilt, roll
            idx_t, th_t = joints[row][8] - joints[row][5], joints[row][4] - joints[row][1] #the real hand
            idx_t, th_t = R @ idx_t, R @ th_t
            idx_t, th_t = idx_t*ratio_idx, th_t*ratio_th

            target_i = idx_t + BY_NAME["index mcp"]["offset"]
            target_t = th_t + BY_NAME["thumb add"]["offset"]
            # initial guess
            print(np.linalg.norm(target_i - BY_NAME["index mcp"]["offset"]))
            q = np.zeros(3)
            g = sim.mj.MjData(model)
            g.qpos[:] = mjdata.qpos
            # solver
            result = minimize(cost, q, args=(target_i, target_t), method='L-BFGS-B', bounds=bounds)
            result2 = minimize(grasp_cost, GRIP_Q, args=(sim, g, theta), method='L-BFGS-B', bounds=grip_bounds)
            results.append([result.fun, result.x])
            print(result.fun, result.x, result.success, result.message)
            print(result2.fun, result2.x, result2.success, result2.message)
            print("target_i from knuckle:", target_i - BY_NAME["index mcp"]["offset"])
            print("target_t from knuckle:", target_t - BY_NAME["thumb mcp"]["offset"])
            steps +=1

            if gripping:
                fingers = result2.x
            else:
                fingers = result.x
            sim.set_ctrl("a_index_mcp", fingers[0])
            sim.set_ctrl("a_thumb_pip", fingers[1])
            sim.set_ctrl("a_thumb_mcp", fingers[2])
            mid_q = mid[row] / (1 + 1.314 + 0.429)
            sim.set_ctrl("a_mid_mcp", mid_q, clip=True)
            sim.set_ctrl("a_mid_pip", 1.314 * mid_q, clip=True)
            sim.set_ctrl("a_mid_mip", 0.429 * mid_q, clip=True)
            sim.set_ctrl("a_wrist_tilt", np.radians(tilt + init_tilt))
            sim.set_ctrl("a_forearm_roll", np.radians(roll + init_roll))
            sim.set_ctrl("a_elbow_pitch", np.radians(elbow_tilt))
            ref_frames.append([np.radians(elbow_tilt), np.radians(init_roll+roll), np.radians(init_tilt+tilt), (fingers[0]), (fingers[1]), (fingers[2]), (mid_q)])
            
            for _ in range(int((1/30) / model.opt.timestep)):
                sim.step()
            frames.append(sim.render(cam, joint_on))
    costs  = np.array([r[0] for r in results])
    solved = np.array([r[1] for r in results])
    grip = np.array([g for g in gripped])
    rows = np.array([r for r in rows])
    np.savez(ROOT / "solved_angles_new2.npz", cost=costs, angles=solved, grip=grip, rows=rows, theta=theta)
    np.savez(ROOT / "refs" / "clip03.npz", q_ref=np.array(ref_frames), fps=30)
    media.write_video(str(ROOT / "real_sim8.mp4"), frames, fps=30)


if __name__ == "__main__":
    GRIP_Q = [0.818, 1.033, -0.291]
    sim = MjcArm()
    model, mjdata = sim.model, sim.data
    g = sim.mj.MjData(model)
    theta = get_theta(GRIP_Q, sim, g)
    main(theta)
