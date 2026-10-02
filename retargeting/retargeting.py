# minimiser implemented using Aleksandar Haber PhD 'Solve OptimizationProblems in Python Using SciPy minimize()
import mediapy as media
import numpy as np
from scipy.optimize import minimize
from arm import ROOT, NPZ_PATH, Calibration, HandPose, MjcArm, RobotArm, DepthScale, ArmIK, STServo, FingerBoard
from main_motion.f_kinematics import BY_NAME, fkine_all

Q = ['a_index_mcp', 'a_thumb_pip', 'a_thumb_mcp']
# leader joints optimised, in Q order
Q_JOINTS = ['index_mcp_joint', 'thumb_pip_joint', 'thumb_mcp_joint']

palm_frame = HandPose.palm_frame
cam_translation = DepthScale.cam_tr
tilting = HandPose.tilting
rolling = HandPose.rolling

duration = 3
framerate = 60


def cost(q: list, t1: float, t2: float):
    q = {"index mcp": q[0], "thumb mcp": q[1], "thumb add": q[2]}
    mx = fkine_all(q)
    i_tip = np.linalg.inv(mx["wrist tilt"]) @ mx["index tip"]
    i_tip = i_tip[:3,3]
    t_tip = np.linalg.inv(mx["wrist tilt"]) @ mx['thumb tip']
    t_tip = t_tip[:3,3]
    return np.linalg.norm(i_tip-t1) + np.linalg.norm(t_tip-t2)


def main():
    data = np.load(NPZ_PATH)
    arm = RobotArm('linux').connect()
    sim = MjcArm()
    f = FingerBoard()
    st = STServo()
    ik = ArmIK(324.3, 100)
    model, mjdata = sim.model, sim.data
    sim.make_renderer()
    print(data.files)

    # joint visualisation
    joint_on = sim.vis_option(False)
    cam = sim.camera(0.7, 60, -20, [0, 0, 0.5])
    sim.reset()
    mjdata.joint("forearm_roll").qpos[0] = np.pi

    joints = data["joints"]
    scales = Calibration.load().hand_scales # robot hand scale ratio
    ratio_idx, ratio_th = scales["index"], scales["thumb"]
    bounds = [sim.joint_range(j) for j in Q_JOINTS]

    angles = np.array([HandPose.apply_angles_arr(joints[i]) for i in range(len(joints))])

    
    results = []
    frames = []
    steps = 0

    normal0, xv0, yv0 = palm_frame(joints[0][5], joints[0][17], joints[0][9], joints[0][0])
    t = DepthScale.cam_tr(data["camera_translation"], data["focal_length"][0])
    mcp_cam = t + joints[:, 9]
    targets = DepthScale.smooth(DepthScale.ik_targets(mcp_cam))
    print("fl:", data["focal_length"][:3])
    print("raw:", data["camera_translation"][:3])
    print("corrected:", t[:3])
    d = np.linalg.norm(targets, axis=1)
    print("target distance mm:", np.nanmin(d), np.nanmax(d))
    print("first targets:", targets[:3])
    normal, xv, yv = palm_frame(joints[0][5], joints[0][17], joints[0][9], joints[0][0]) # changed for tests
    init_tilt = tilting(yv0, yv, normal0)
    init_roll = rolling(xv, normal0, normal)

    for row in range(100):

        if np.isnan(joints[row]).any():
            continue
        else:
            print("target:", targets[row])
            result = ik.solve(targets[row])   
            elbow_tilt = 0.0       # ← here
            if result is not None:
                elbow_tilt = result[1]
            normal, xv, yv = palm_frame(joints[row][5], joints[row][17], joints[row][9], joints[row][0])
            print(elbow_tilt)
            R = np.array([-normal, yv, xv])
            tilt = tilting(yv0, yv, normal0)
            roll = rolling(xv, normal0, normal)
            idx_t, th_t = joints[row][8] - joints[row][5], joints[row][4] - joints[row][1] #the real hand
            idx_t, th_t = R @ idx_t, R @ th_t
            idx_t, th_t = idx_t*ratio_idx, th_t*ratio_th

            target_i = idx_t + BY_NAME["index mcp"]["offset"]
            target_t = th_t + BY_NAME["thumb add"]["offset"]
            # initial guess
            print(np.linalg.norm(target_i - BY_NAME["index mcp"]["offset"]))
            q = np.zeros(3)
            # solver
            result = minimize(cost, q, args=(target_i, target_t), method='L-BFGS-B', bounds=bounds)
            results.append([result.fun, result.x])
            print(result.fun, result.x, result.success, result.message)
            print("target_i from knuckle:", target_i - BY_NAME["index mcp"]["offset"])
            print("target_t from knuckle:", target_t - BY_NAME["thumb mcp"]["offset"])
            steps +=1

            sim.set_ctrl("a_index_mcp", result.x[0])
            sim.set_ctrl("a_thumb_pip", result.x[1])
            sim.set_ctrl("a_thumb_mcp", result.x[2])
            sim.set_ctrl("a_wrist_tilt", np.radians(tilt + init_tilt))
            sim.set_ctrl("a_forearm_roll", np.radians(roll + init_roll))
            sim.set_ctrl("a_elbow_pitch", np.radians(elbow_tilt))
            
            #arm.wrist_rotator(st.sim_to_real(np.radians(roll + init_roll), dir), speed, accel))
            #arm.wrist_tilter(st.sim_to_real(np.radians(tilt + init_tilt), dir), speed, accel))
            #arm.elbow(st.sim_to_real(np.radians(elbow_tilt, dir), speed, accel))
            for i in range (2,5):
                us = f.sim_to_real(i, angles[row][i])
                f.move_us(i, us)
            us_i, us_t, us_a = f.sim_to_real(5, result.x[0]), f.sim_to_real(1, result.x[1]), f.sim_to_real(0, result.x[2])
            f.move_us(5, us_i)
            f.move_us(1, us_t)
            f.move_us(0, us_a)
            for _ in range(int((1/30) / model.opt.timestep)):
                sim.step()
                frames.append(sim.render(cam, joint_on))
    costs  = np.array([r[0] for r in results])
    solved = np.array([r[1] for r in results])
    np.savez(ROOT / "solved_angles_new2.npz", cost=costs, angles=solved)
    media.write_video(str(ROOT / "real_sim6.mp4"), frames, fps=30)


if __name__ == "__main__":
    main()
