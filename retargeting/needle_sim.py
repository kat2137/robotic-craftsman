# minimiser implemented using Aleksandar Haber PhD 'Solve Optimization Problems in Python Using SciPy minimize()
import mediapy as media
import numpy as np
from scipy.optimize import minimize
from arm import ROOT, NPZ_PATH, XML_PATH, Calibration, HandPose, MjcArm, DepthScale, ArmIK, STServo, FingerBoard
from main_motion.f_kinematics import BY_NAME, fkine_all
from retargeting.fabric_sim import fabric_force_model

Q = ['a_index_mcp', 'a_thumb_pip', 'a_thumb_mcp']
# leader joints optimised, in Q order
Q_JOINTS = ['index_mcp_joint', 'thumb_pip_joint', 'thumb_mcp_joint']

W = 30
H = 0.001
comp_value = 0.01
PIERCE_TIMES = [(6, 8, 0.2), (15, 17, 0.6), (32, 33, 0.4), (38, 40, 0.5)] # marked times when needle is piercing the fabric
pierce_period = []
fps = 30
framerate = 60
duration = 3
ref_frames = []
t1 = 0.012

for st, et, depth in PIERCE_TIMES:
    frame_st, frame_end = round((st - 0) * fps), round((et - 0)* fps)
    final = 0.001 + H + depth * 0.0738
    pierce_period.append((frame_st, frame_end, final))


palm_frame = HandPose.palm_frame
cam_translation = DepthScale.cam_tr
tilting = HandPose.tilting
rolling = HandPose.rolling



def cost(q: list, t1: float, t2: float):
    q = {"index mcp": q[0], "thumb mcp": q[1], "thumb add": q[2]}
    mx = fkine_all(q)
    i_tip = np.linalg.inv(mx["wrist tilt"]) @ mx["index tip"]
    i_tip = i_tip[:3,3]
    t_tip = np.linalg.inv(mx["wrist tilt"]) @ mx['thumb tip']
    t_tip = t_tip[:3,3]
    return np.linalg.norm(i_tip-t1) + np.linalg.norm(t_tip-t2)   
    
def ntip_distance(x: list, sim, g, depth_local: float, n: float):
    for i, joint in zip(['elbow_pitch', 'wrist_tilt'], x):
         sim.set_leader(i, joint, g)
    sim.kinematics(g)
    tid = sim.model.body("thumb_tip").id
    t_tip = g.xpos[tid]
    fid = sim.model.geom("fabric_geom").id
    p0 = sim.data.geom_xpos[fid]
    f_face = p0 - (H/2)*n
    tip = t_tip + g.xmat[tid].reshape(3,3) @ depth_local
    return np.dot(tip - f_face, n), tip

def ntip_cost(x: list, x_ret: list, sim, g, depth: float, target_depth:float, n: float, Fm, p0):
    distance, tip = ntip_distance(x, sim, g, depth, n)
    x_result = (x[0] - x_ret[0])**2 + (x[1] - x_ret[1])**2
    a = np.dot(tip - p0, Fm[:, 0])
    b = np.dot(tip - p0, Fm[:, 1])
    outside = max(0, abs(a) - 0.090)**2 + max(0, abs(b) - 0.125)**2
    return (distance - target_depth)**2 + x_result * comp_value + outside * 1000

def grip_detect(d:float, was_gripping:bool , t_on:float , t_off:float):
    if was_gripping:
        return d < t_off      
    return d < t_on           

def touch_solver(sim, g, theta):
    i_d = g.site_xmat[sim.site_id('index_touch_site')].reshape(3, 3)[:, 2]
    t_d = g.site_xmat[sim.site_id("thumb_touch_site")].reshape(3, 3)[:, 2]
    s_d = g.site_xmat[sim.site_id("mid_support_site")].reshape(3,3)[:,2]
    angle = np.arccos(np.clip(-np.dot(i_d, t_d), -1, 1))
    return (angle - theta) ** 2

def get_theta(GRIP_Q, sim, g):
    print("grip gap mm:", pad_gap(GRIP_Q, sim, g) * 1000)
    i_d = g.site_xmat[sim.site_id('index_touch_site')].reshape(3, 3)[:, 2]
    t_d = g.site_xmat[sim.site_id("thumb_touch_site")].reshape(3, 3)[:, 2]
    s_d = g.site_xmat[sim.site_id("mid_support_site")].reshape(3,3)[:,2]
    return np.arccos(np.clip(-np.dot(i_d, t_d), -1, 1))

def pad_gap(q:list, sim, g):
    for j, val in zip(Q_JOINTS, q):
        sim.set_leader(j, val, g)
    sim.kinematics(g)
    return np.linalg.norm(sim.site_pos("index_touch_site", g) - sim.site_pos("thumb_touch_site", g))

MIN_GAP = 2.0   #needle diameter
def grasp_cost(q, sim, g, theta):
    gap = (pad_gap(q, sim, g) * 1000 - MIN_GAP) ** 2
    return gap + W * touch_solver(sim, g, theta)

def needle_model(sim):
    spec = sim.mj.MjSpec.from_file(str(XML_PATH))
    spec.geom("needle_geom").contype = 2
    spec.geom("needle_geom").conaffinity = 2
    spec.geom("floor").contype = 3
    spec.geom("floor").conaffinity = 3
    eq = spec.add_equality()
    eq.name = "needle_grip"
    tip = spec.body("needle").add_site(name="needle_tip", pos=[0.1769, 0.0990, 0.0010])
    eq.type = sim.mj.mjtEq.mjEQ_WELD
    eq.objtype = sim.mj.mjtObj.mjOBJ_BODY
    eq.name1 = "thumb_tip"
    eq.name2 = "needle"
    eq.active = False
    eq.solref = [0.005, 1]
    sim.model = spec.compile()
    sim.data = sim.mj.MjData(sim.model)


def mid_clearance(sim, ng, mg):
    fromto = np.zeros(6)
    return sim.mj.mj_geomDistance(sim.model, sim.data, ng, mg, 0.05, fromto)


def grip_needle(sim):
    model, mjdata = sim.model, sim.data
    eq = model.equality("needle_grip").id
    nb = model.body("needle").id
    tb = model.body("thumb_tip").id
    anchor = model.geom_pos[model.geom("needle_geom").id]          
    point = mjdata.xpos[nb] + mjdata.xmat[nb].reshape(3, 3) @ anchor
    Rt = mjdata.xmat[tb].reshape(3, 3)
    q_inv = np.zeros(4)
    sim.mj.mju_negQuat(q_inv, mjdata.xquat[tb])
    rel_q = np.zeros(4)
    sim.mj.mju_mulQuat(rel_q, q_inv, mjdata.xquat[nb])               
    model.eq_data[eq][0:3] = anchor
    model.eq_data[eq][3:6] = Rt.T @ (point - mjdata.xpos[tb])       
    model.eq_data[eq][6:10] = rel_q
    mjdata.eq_active[eq] = 1


def release_needle(sim):
    sim.data.eq_active[sim.model.equality("needle_grip").id] = 0


def place_needle(sim):
    model, mjdata = sim.model, sim.data
    pad_i = sim.site_pos("index_touch_site")
    pad_t = sim.site_pos("thumb_touch_site")
    pad_s = sim.site_pos('mid_support_site')
    P = (pad_i + pad_t)/2
    u = (pad_t - pad_i)/np.linalg.norm(pad_t - pad_i)
    v = P - pad_s
    a = 0.05
    x = v - np.dot(v, u) * u
    x = x / np.linalg.norm(x)
    z = np.cross(-x, u)
    R = np.column_stack([-x, u, z])
    nc = model.geom("needle_geom").id        # capsule: its offset positions the needle
    ng = model.geom("needle_vis").id         # real needle shape, for the clearance check
    mg = model.geom("mid_pad_vis").id        # real middle cap shape
    adr = model.jnt_qposadr[model.joint("needle_free").id]
    quat = np.zeros(4)
    sim.mj.mju_mat2Quat(quat, R.flatten())
    mjdata.qpos[adr + 3:adr + 7] = quat
    S_on = P - np.dot(P - pad_s, x) * x
    eye = S_on - 0.008 * x
    mid = eye + 0.0369 * x
    mjdata.qpos[adr:adr+3]= mid - R @ model.geom_pos[nc]
    sim.mj.mj_forward(model, mjdata)
    dist = mid_clearance(sim, ng, mg)
    k = 0
    while dist < -0.0002 and k * a < 0.5:
        k += 1
        x_new = np.cos(k * a) * x + np.sin(k * a)* np.cross(u, x)
        R = np.column_stack([-x_new, u, np.cross(-x_new, u)])
        S_on = P - np.dot(P - pad_s, x_new) * x_new
        eye = S_on - 0.008 * x_new
        mid = eye + 0.0369 * x_new
        sim.mj.mju_mat2Quat(quat, R.flatten())
        mjdata.qpos[adr + 3:adr + 7] = quat
        mjdata.qpos[adr:adr+3]= mid - R @ model.geom_pos[nc]
        sim.mj.mj_forward(model, mjdata)
        dist = mid_clearance(sim, ng, mg)
    print(f"needle placed: rotation steps {k}, middle cap clearance {dist * 1000:.2f} mm")
    mjdata.qvel[model.jnt_dofadr[model.joint("needle_free").id]:][:6] = 0
    grip_needle(sim)

def pierce_target(row: int):
    for st, ed, depth in pierce_period:
        if row >= st and row <= ed:
            return (row - st) / (ed - st) * depth
    return None


def main(theta):
    data = np.load(NPZ_PATH)
    sim = MjcArm()
    needle_model(sim)
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
    mjdata.xfrc_applied[model.body("needle").id] = [0, 0, -0.0005 * 9.81, 0, 0, 0]
    mjdata.joint("forearm_roll").qpos[0] = np.pi
    # park the needle away from the arm until the first pinch places it
    n_adr = model.jnt_qposadr[model.joint("needle_free").id]
    n_dof = model.jnt_dofadr[model.joint("needle_free").id]
    mjdata.qpos[n_adr:n_adr + 3] = [0.5, 0.5, 0.05]
    mjdata.qvel[n_dof:n_dof + 6] = 0
    sim.mj.mj_forward(model, mjdata)
    pad_i = sim.site_pos("index_touch_site")
    pad_t = sim.site_pos("thumb_touch_site")
    print(model.dof_damping[model.jnt_dofadr[model.joint("needle_free").id]:][:6])
    print(model.dof_armature[model.jnt_dofadr[model.joint("needle_free").id]:][:6])

    joints = data["joints"]
    scales = Calibration.load().hand_scales # robot hand scale ratio
    ratio_idx, ratio_th = scales["index"], scales["thumb"]
    bounds = [sim.joint_range(j) for j in Q_JOINTS]
    grip_bounds = list(bounds)
    grip_bounds[1] = (grip_bounds[1][0], 0.1)   
    arm_bounds = [sim.joint_range('elbow_pitch'), sim.joint_range('wrist_tilt')]

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

    # flags for needle re-placement
    gripping = False
    prev_grip = False
    pending = False
    state = {'contact': None, "pierced": False}
    prev_tip = sim.site_pos("needle_tip")

    tip_d = np.linalg.norm(joints[:, 8] - joints[:, 4], axis=1)
    win = np.lib.stride_tricks.sliding_window_view(np.pad(tip_d, 2, mode='edge'), 5)
    tip_d = np.nanmedian(win, axis=1)  # 5-frame median, NaN-safe
    eq = model.equality("needle_grip").id
    tb = model.body("thumb_tip").id
    fid = model.geom("fabric_geom").id
    Fm = sim.data.geom_xmat[fid].reshape(3, 3)
    n = Fm[:, 2]
    p0 = sim.data.geom_xpos[fid]
          
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
            elbow_cmd, tilt_cmd, roll_cmd = np.radians(elbow_tilt), np.radians(tilt + init_tilt), np.radians(roll + init_roll)

            if gripping:
                fingers = result2.x  
            else:
                fingers = result.x
            target_depth = pierce_target(row)
            if target_depth is not None and mjdata.eq_active[eq] == 1:
                x_ret = [np.radians(elbow_tilt), np.radians(tilt + init_tilt)]
                R_t = mjdata.xmat[tb].reshape(3, 3) # rotation matrix for the thumb
                tip_local = R_t.T @ (sim.site_pos("needle_tip") - mjdata.xpos[tb])
                g.qpos[:] = mjdata.qpos
                result3 = minimize(ntip_cost, x_ret, args = (x_ret, sim, g, tip_local, target_depth, n, Fm, p0), method='L-BFGS-B', bounds=arm_bounds)
                elbow_cmd, tilt_cmd = result3.x
                print (result3.x, result3.fun)
            sim.set_ctrl("a_index_mcp", fingers[0])
            sim.set_ctrl("a_thumb_pip", fingers[1])
            sim.set_ctrl("a_thumb_mcp", fingers[2])
            mid_q = mid[row] / (1 + 1.314 + 0.429)
            sim.set_ctrl("a_mid_mcp", mid_q, clip=True)
            sim.set_ctrl("a_mid_pip", 1.314 * mid_q, clip=True)
            sim.set_ctrl("a_mid_mip", 0.429 * mid_q, clip=True)
            sim.set_ctrl("a_wrist_tilt", (tilt_cmd))
            sim.set_ctrl("a_forearm_roll", (roll_cmd))
            sim.set_ctrl("a_elbow_pitch", (elbow_cmd))
            ref_frames.append([elbow_cmd, roll_cmd, tilt_cmd, (fingers[0]), (fingers[1]), (fingers[2]), (mid_q)])
            print(f'Tip_d:{tip_d[row]}')
            # once per frame: grip onset -> wait for the pinch to close; grip end -> let go
            if gripping and not prev_grip:
                pending = True
            if not gripping:
                pending = False
                #release_needle(sim)
            prev_grip = gripping
            nb = model.body("needle").id
            for _ in range(int((1/30) / model.opt.timestep)):
                P, n, tip, L = fabric_force_model(sim, state, prev_tip)
                force = -P * n
                print(f'P:{P}, n: {n}, tip:{tip}, N/m:{force}, state:{state["contact"]}')
                mjdata.qfrc_applied[:] = 0
                sim.mj.mj_applyFT(model, mjdata, force, np.zeros(3), tip, tb, mjdata.qfrc_applied)
                sim.step()
                prev_tip = sim.site_pos("needle_tip").copy()
                if pending:
                    gap = np.linalg.norm(sim.site_pos("index_touch_site") - sim.site_pos("thumb_touch_site"))
                    if gap < 0.008:
                        place_needle(sim)
                        pending = False
                        state["contact"] = None
                        state["pierced"] = False
                        prev_tip = sim.site_pos("needle_tip").copy()
                        frames.append(sim.render(cam, joint_on))   # spawn frame
            frames.append(sim.render(cam, joint_on))
    costs  = np.array([r[0] for r in results])
    solved = np.array([r[1] for r in results])
    grip = np.array([g for g in gripped])
    rows = np.array([r for r in rows])
    np.savez(ROOT / "solved_angles_frame1.npz", cost=costs, angles=solved, grip=grip, rows=rows, theta=theta)
    np.savez(ROOT / "refs" / "clip05.npz", q_ref=np.array(ref_frames), fps=30)
    media.write_video(str(ROOT / "needle_test2.mp4"), frames, fps=30)


if __name__ == "__main__":
    GRIP_Q = [0.818, 1.033, -0.291]
    sim = MjcArm()
    model, mjdata = sim.model, sim.data
    g = sim.mj.MjData(model)
    theta = get_theta(GRIP_Q, sim, g)
    main(theta)