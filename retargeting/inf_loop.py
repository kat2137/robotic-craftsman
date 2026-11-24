# this file was written using Open Duck inference example on github https://github.com/apirrone/Open_Duck_Mini_Runtime/blob/v2/scripts/v2_rl_walk_mujoco.py
import time
import numpy as np
import os
import pickle
from arm import RobotArm, MjcArm, FingerBoard, STServo, GEAR_RATIO, COUNTS_PER_DEG
from retargeting.needle_sim import needle_model
CTRL_DT = 1/30

class RLPath:
    def __init__(self, policy_path: str,  serial_port: str = "/dev/ttyACM0", timer = CTRL_DT):


        self.ST = [(0, 3, -1), (1, 1, 1), (2, 2, 1)]#(column, servo id, direction)
        self.ORDER = [5, 1, 0, 2]                             
        self.RANGES = {5: (-0.05, 1.2), 1: (-0.05, 1.25), 0: (0.1, -1.27), 2: (0.0, 1.6)}
        
        with open(policy_path) as f:
            exp = pickle.load(f)

        q_ref = exp["q_ref"]
        self.Q_REF = q_ref[:, :6]        # elbow, roll, tilt, index, thumb pip, thumb mcp
        self.Q_PASSIVE = q_ref[:, 6:]
        
        self.obs_mean = exp["obs_mean"]
        self.obs_std = exp["obs_std"]
        self.policy_params = exp["policy"]
        self.ACTION_SCALE = exp["action_scale"]
        self.LOWERS = exp["lowers"]
        self.UPPERS = exp["uppers"]
        self.CTRL_DT = exp["ctrl_dt"]
        self.PIERCE = exp["pierce_depth"]
        self.speed = CTRL_DT


        self.arm = RobotArm('linux').connect()
        self.t0, self.t = self.Q_REF[0, :], 0
        self.last_action = np.zeros(6)
        self.finger_cmd = np.zeros(6)
        self.layers = exp["layers"]
        self.prev_qpos = self.Q_REF[0].copy()
        

        self.save_obs = True
        if self.save_obs:
            self.saved_obs = []

        self.sim = MjcArm()
        needle_model(self.sim)
        self.g = self.sim.mj.MjData(self.sim.model)
        R_rel = np.zeros(9)
        self.sim.mj.mju_quat2Mat(R_rel, W[6:10])
        R_rel = R_rel.reshape(3, 3)
        m, W = self.sim.model, exp["needle_weld"]
        rel_pos = W[3:6] - R_rel @ W[0:3]                       
        site_pos = m.site_pos[m.site("needle_tip").id]
        self.tip_local = rel_pos + R_rel @ site_pos
        self.sim.mj.mj_forward(m, self.sim.data)
        fid = m.geom("fabric_geom").id
        self.n = self.sim.data.geom_xmat[fid].reshape(3, 3)[:, 2]
        self.f_face = self.sim.data.geom_xpos[fid] - (H / 2) * self.n
        self.tb = m.body("thumb_tip").id


    def send(self, target, mid):
        for col, idm, dirc in self.ST:
            counts = STServo.sim_to_real(idm, target[col], dirc)
            self.arm.st(idm).write(counts, self.speed, self.accel)
        for val, ch in zip([*target[3:6], mid], self.ORDER):
            straight, flexed = self.RANGES[ch]
            frac = np.clip((val - straight) / (flexed - straight), 0, 1)
            self.arm.fingers.move_finger(ch, self.arm.fingers.servos[ch].frac_to_us(frac))

    def policy(self, obs):
        x = (obs - self.obs_mean)/ self.obs_std
        for i, (W,b) in enumerate(self.layers):
            x = x @ W + b
            if i < len(self.layers) - 1:
                x = x / (1+np.exp(-x))
        return np.tanh(x[:6])

    def get_obs(self):
        qpos = np.zeros(6)
        qvel = np.zeros(6)
        for col, idm, dirc in range (self.ST):
            counts, speed, comm, err = self.arm.st(idm).packetHandler.ReadPosSpeed(idm)
            qpos[col] = np.radians((counts - STServo.ZERO_COUNTS[idm]) / (dirc * GEAR_RATIO[idm] * COUNTS_PER_DEG))
        qpos[1] += np.pi
        qpos[3:6] = self.finger_cmd[:3]
        for i in range (len(qpos)):
            qvel = (qpos - self.prev_qpos) / CTRL_DT
            self.prev_qpos = qpos

            depth = self.needle_depth(qpos)
            target_d = self.PIERCE[self.t + 1]
            flag = 1.0 if target_d >= 0 else 0.0
        return np.concatenate([qpos, qvel, self.last_action, self.Q_REF[self.t + 1] - qpos, [depth, target_d, flag]]) 

    def needle_depth(self, qpos):
        for joint, val in zip(['elbow_pitch', 'forearm_roll', 'wrist_tilt', 'index_mcp_joint', 'thumb_pip_joint', 'thumb_mcp_joint'], qpos):
            self.sim.set_leader(joint, val, self.g)
        self.sim.kinematics(self.g)
        tip = self.g.xpos[self.tb]
        return np.dot(tip - self.f_face, self.n)


    def start(self):
        
        time.sleep(2)

    