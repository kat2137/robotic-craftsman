"""Core classes for the robotic craftsman arm.

Every other module imports its hardware, calibration, kinematics, simulation and
camera helpers from here.

Nothing in this file touches hardware at import time. scservo_sdk, the PCA9685
libraries, board/busio, mujoco and cv2 are imported inside the methods that need
them, so `import arm` works on a Mac with nothing plugged in.

Channel naming (confirmed on video 10 Sept):
    ch0 = thumb adduction ("thumb_add")
    ch1 = thumb flexion   ("thumb")
    ch5 = index           ("index")
"""

import glob
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal, Optional

import numpy as np

# --- paths -------------------------------------------------------------------

ROOT = Path(__file__).resolve().parent
FINGER_CALIB_PATH = ROOT / "finger_calib.json"
RATIOS_PATH = ROOT / "main_motion" / "ratios.json"
SCALES_PATH = ROOT / "main_motion" / "scale_retarget.json"
XML_PATH = ROOT / "model" / "palm_with_frame.xml"
NPZ_PATH = ROOT / "handsewing_01.npz"

# --- ST bus servos (wrist rotate, wrist tilt, elbow) ------------------------

ST_PORT_LINUX = "/dev/ttyACM0"
ST_BAUD = 1000000
ST_BROADCAST_ID = 0xFE        # 254 = broadcast
COUNTS_PER_DEG = 4096 / 360
GEAR_RATIO = {1:1.0, 2:1.0, 3:2.0}   # STS3215: 4096 counts per turn


ST_SERVOS = {
    "wrist rotate": {"id": 1, "pos": 2059, "straight": 2048, "flexed": 4095, "min": 2000, "max": 4095},
    "wrist tilt":   {"id": 2, "pos": 2043, "straight": 2048, "flexed": 4095, "min": 2200, "max": 3000},
    "elbow":        {"id": 3, "pos": 3052, "straight": 2048, "flexed": 4095, "min": None, "max": None},
}

# --- PCA9685 finger servos ---------------------------------------------------

PWM_FREQ = 60
US_MIN, US_MID, US_MAX = 1000, 1500, 2000
LEGACY_US_PER_COUNT = 4.07            # Adafruit_PCA9685 set_pwm path, 60 Hz
LEGACY_COUNT_MIN, LEGACY_COUNT_MAX = 100, 600

# Order of the six angle columns from HandPose.apply_angles_arr
FINGER_ORDER = ["thumb_add", "thumb", "middle", "ring", "pinkie", "index"]

# Channels not yet in finger_calib.json (it only holds the two pinching fingers).
EXTRA_FINGERS = {
    0: {"name": "thumb add","straight":2200, "flexed": 800},
    1: {"name": "thumb",  "straight": 2000,   "flexed": 800},
    2: {"name": "middle", "straight": 1500, "flexed": 2400},
    3: {"name": "ring",   "straight": 1600, "flexed": 500},
    4: {"name": "pinkie", "straight": 1700, "flexed": 1000},
    5: {"name": "index",  "straight":  2200, "flexed": 1300 }
}


# =============================================================================
# Calibration
# =============================================================================

@dataclass
class Calibration:
    """finger_calib.json + ratios.json (w_spring) + scale_retarget.json in one place.

    Joint limits in radians and the MJCF coupling ratios live in the MuJoCo model;
    read them with MjcArm.joint_range() / MjcArm.couplings().
    """
    finger_calib: dict
    ratios: dict
    scales: dict

    @classmethod
    def load(cls, finger_path=FINGER_CALIB_PATH, ratios_path=RATIOS_PATH, scales_path=SCALES_PATH):
        with open(finger_path) as f:
            finger_calib = json.load(f)
        with open(ratios_path) as f:
            ratios = json.load(f)
        with open(scales_path) as f:
            scales = json.load(f)
        return cls(finger_calib, ratios, scales)

    # finger_calib.json
    @property
    def channels(self) -> dict:
        """Raw channel table, keyed by channel string: "0" -> {name, position, ...}."""
        return self.finger_calib["channels"]

    @property
    def wrist_tilt_ref(self) -> int:
        return self.finger_calib["wrist_tilt_ref"]

    @property
    def wrist_id(self) -> int:
        return self.finger_calib.get("wrist_id", 2)

    @property
    def wrist_speed(self) -> int:
        return self.finger_calib.get("wrist_speed", 100)

    @property
    def wrist_accel(self) -> int:
        return self.finger_calib.get("wrist_accel", 100)

    @property
    def tenodesis_k(self) -> float:
        """P_f = T_f + k * (P_w - P_w0)"""
        return self.finger_calib.get("tenodesis_k", 0.02)

    @property
    def st_port(self) -> str:
        return self.finger_calib.get("st_port", ST_PORT_LINUX)

    @property
    def st_baud(self) -> int:
        return self.finger_calib.get("st_baud", ST_BAUD)

    @property
    def pwm_freq(self) -> int:
        return self.finger_calib.get("pwm_freq", PWM_FREQ)

    def fingers(self) -> dict:
        """All finger servos keyed by int channel: finger_calib.json + EXTRA_FINGERS."""
        out = {ch: FingerServo(ch=ch, **cfg) for ch, cfg in EXTRA_FINGERS.items()}
        for ch, cfg in self.channels.items():
            out[int(ch)] = FingerServo(ch=int(ch), **cfg)
        return out

    # ratios.json
    @property
    def w_spring(self) -> dict:
        return self.ratios["w_spring"]

    def coupling_ratios(self) -> dict:
        """Follower / leader ratios from the measured curl fractions."""
        idx, thb = self.w_spring["5"], self.w_spring["1"]
        return {
            "index_pip": idx["pip"] / idx["mcp"],
            "index_dip": idx["dip"] / idx["mcp"],
            "thumb_ip":  thb["pip"] / thb["mcp"],
        }

    @property
    def thumb_add_deg_per_us(self) -> float:
        return self.w_spring["0"]["degrees per 100us"] / 100

    # scale_retarget.json
    @property
    def hand_scales(self) -> dict:
        """Robot / human finger length scale, e.g. {"index": 1.128, "thumb": 1.079}."""
        return {"index": self.scales["index"][0], "thumb": self.scales["thumb"][0]}

    @property
    def robot_lengths(self) -> dict:
        return {"index": self.scales["index"][1], "thumb": self.scales["thumb"][1]}


# =============================================================================
# ST bus servos
# =============================================================================

def _sdk():
    import scservo_sdk
    return scservo_sdk


class STBus:
    """One serial port shared by all ST bus servos."""

    def __init__(self):
        self.portHandler = None
        self.packetHandler = None

    @staticmethod
    def find_port(environment: Literal['mac', 'linux']) -> str:
        if environment == 'mac':
            ports = sorted(glob.glob('/dev/tty.usbmodem*') + glob.glob('/dev/tty.usbserial*'))
            if not ports:
                raise RuntimeError("No USB serial device found - is the servo board plugged in?")
            return ports[0]
        return ST_PORT_LINUX

    def open_port(self, port: str = ST_PORT_LINUX) -> bool:
        sdk = _sdk()
        self.portHandler = sdk.PortHandler(port)
        self.packetHandler = sdk.sms_sts(self.portHandler)
        return self.portHandler.openPort()

    def set_baud(self, baud: int = ST_BAUD) -> bool:
        return self.portHandler.setBaudRate(baud)

    def connect(self, port: str = ST_PORT_LINUX, baud: int = ST_BAUD) -> bool:
        """Open + set baud, printing the same messages the old scripts did."""
        if self.open_port(port):
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            return False
        if self.set_baud(baud):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            return False
        return True

    def close(self):
        self.portHandler.closePort()

    def servo(self, name: str) -> "STServo":
        cfg = ST_SERVOS[name]
        return STServo(id=cfg["id"], pos=cfg["pos"], straight=cfg["straight"], flexed=cfg["flexed"],
                       portHandler=self.portHandler, min=cfg["min"], max=cfg["max"])

    def servo_by_id(self, servo_id: int) -> "STServo":
        for name, cfg in ST_SERVOS.items():
            min, max = cfg["min"], cfg["max"]
            if cfg["id"] == servo_id:
                return self.servo(name)
        return STServo(id=servo_id, pos=0, straight=0, flexed=0, portHandler=self.portHandler)

    def ping_scan(self, ids=range(1, 21)) -> list:
        """[(id, model, pos)] for every servo that answers."""
        sdk = _sdk()
        found = []
        for sid in ids:
            model, comm, err = self.packetHandler.ping(sid)
            if comm == sdk.COMM_SUCCESS:
                pos, comm2, err2 = self.packetHandler.ReadPos(sid)
                found.append((sid, model, pos))
        return found

    def broadcast(self, position: int, speed: int, acceleration: int):
        return self.packetHandler.WritePosEx(ST_BROADCAST_ID, position, speed, acceleration)


@dataclass
class STServo:
    id: int
    pos: int
    straight: int
    flexed: int
    portHandler: object
    min: Optional[int] = None
    max: Optional[int] = None
    ZERO_COUNTS = {
        1: 2000,
        2: 3000,
        3: 3052
        }

    def __post_init__(self):
        self.packetHandler = _sdk().sms_sts(self.portHandler)

    @staticmethod
    def counts_to_deg(counts: float) -> float:
        return counts / COUNTS_PER_DEG

    @staticmethod
    def deg_to_counts(deg: float) -> float:
        return deg * COUNTS_PER_DEG

    def write(self, position: int, speed: int, acceleration: int) -> bool:
        """Send the goal position. Returns False if the packet did not get through."""
        sdk = _sdk()
        scs_comm_result, scs_error = self.packetHandler.WritePosEx(self.id, position, speed, acceleration)
        if scs_comm_result != sdk.COMM_SUCCESS:
            print(f"{self.packetHandler.getTxRxResult(scs_comm_result)}")
            return False
        elif scs_error != 0:
            print(f"{self.packetHandler.getRxPacketError(scs_error)}")
        return True

    def move(self, position: int, speed: int, acceleration: int):
        """Write the goal position, then block until the servo stops."""
        self.write(position, speed, acceleration)
        self.wait()

    def wait(self):
        """Block until the servo stops, printing position and speed on every poll."""
        sdk = _sdk()
        while 1:
            # Read the current position of servo(ID)
            scs_present_position, scs_present_speed, scs_comm_result, scs_error = self.packetHandler.ReadPosSpeed(self.id)
            if scs_comm_result != sdk.COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(scs_comm_result))
            else:
                self.pos = scs_present_position
                print(f"[ID:{self.id:03d}] PresPos:{scs_present_position} PresSpd:{scs_present_speed}")
            if scs_error != 0:
                print(self.packetHandler.getRxPacketError(scs_error))

            # Read moving status of servo(ID)
            moving, scs_comm_result, scs_error = self.packetHandler.ReadMoving(self.id)
            if scs_comm_result != sdk.COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(scs_comm_result))

            if moving == 0:
                break
        return

    def wait_until_stopped(self, timeout: float = 6.0, poll: float = 0.02, settle: float = 0.15):
        """Quiet version of wait() with a deadline. Returns the last position read, or None."""
        import time
        sdk = _sdk()
        time.sleep(settle)
        deadline = time.time() + timeout
        reached = None
        while time.time() < deadline:
            pos, speed, comm, err = self.packetHandler.ReadPosSpeed(self.id)
            if comm != sdk.COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(comm))
            elif err != 0:
                print(self.packetHandler.getRxPacketError(err))
            else:
                reached = pos

            moving, comm, err = self.packetHandler.ReadMoving(self.id)
            if comm != sdk.COMM_SUCCESS:
                print(self.packetHandler.getTxRxResult(comm))
            elif moving == 0:
                return reached

            time.sleep(poll)

        print(f"timeout waiting for servo {self.id}")
        return reached
    @staticmethod
    def sim_to_real(id:int, angle_rad:float, dir:int):
            if np.isnan(angle_rad):
                raise ValueError(f"invalid angle value")
            if dir not in (-1, 1):
                raise ValueError(f"Direction needs to be either 1 or -1.")
            if id not in STServo.ZERO_COUNTS:
                raise ValueError(f"no zero calibration for servo {id}")
            else:
                counts = STServo.ZERO_COUNTS[id] + dir * GEAR_RATIO[id] * np.degrees(angle_rad) * COUNTS_PER_DEG
                if not 0 <= counts <= 4095:
                    raise ValueError(f"Katarzyna, these counts are unacceptable! Your robot can't do these!")
                for ix, val in ST_SERVOS.items():
                    if val["id"] == id:
                        lo, hi = val["min"], val["max"]
                        if lo is None and hi is None:
                            continue
                        else: 
                            print('Joint has a set range limit')
                        return int(round(np.clip(counts, lo, hi)))
            return int(round(counts))


# =============================================================================
# PCA9685 finger servos
# =============================================================================

@dataclass
class FingerServo:
    ch: int
    name: str
    position: Optional[int] = None   # grasp target, us
    release: Optional[int] = None    # release target, us
    straight: Optional[int] = None   # us at straight
    flexed: Optional[int] = None     # us at full flex

    @property
    def limits(self) -> tuple:
        """(lo, hi) travel limits in us, ordered low-to-high."""
        return min(self.straight, self.flexed), max(self.straight, self.flexed)

    def frac_to_us(self, frac: float) -> float:
        """0.0 = straight .. 1.0 = flexed."""
        return self.straight + frac * (self.flexed - self.straight)


class FingerBoard:
    """PCA9685 board driving the finger servos.

    backend="circuitpython" -> adafruit_pca9685 over board/busio, duty_cycle in 16 bit
    backend="legacy"        -> Adafruit_PCA9685 on busnum=1, set_pwm in 12 bit counts
    """

    def __init__(self, calib: Optional[Calibration] = None, freq: int = PWM_FREQ,
                 backend: Literal["circuitpython", "legacy"] = "circuitpython"):
        self.calib = calib if calib is not None else Calibration.load()
        self.freq = freq
        self.backend = backend
        self.servos = self.calib.fingers()
        self.by_name = {s.name: s for s in self.servos.values()}
        self.pca = None
        self.pwm = None

    def connect(self):
        if self.backend == "circuitpython":
            from board import SCL, SDA
            import busio
            from adafruit_pca9685 import PCA9685
            i2c = busio.I2C(SCL, SDA)
            self.pca = PCA9685(i2c)
            self.pca.frequency = self.freq
        else:
            from Adafruit_PCA9685 import PCA9685
            self.pwm = PCA9685(busnum=1)
            self.pwm.set_pwm_freq(self.freq)
        return self

    # circuitpython path
    def us_to_duty(self, us):
        period_us = 1_000_000 / self.freq
        return int(us / period_us * 65535)

    def move_us(self, ch, us, verbose=True):
        if verbose:
            print(f"ch {ch}: {us} us")
        self.pca.channels[ch].duty_cycle = self.us_to_duty(us)

    def release_channel(self, ch):
        self.pca.channels[ch].duty_cycle = 0

    # legacy path
    def move_to_angle(self, name, position, angle_min, angle_max):
        """Map a hand angle in [angle_min, angle_max] onto the servo's straight..flexed range."""
        s = self.by_name[name]
        frac = float(np.clip((position - angle_min) / (angle_max - angle_min), 0.0, 1.0))
        us = s.frac_to_us(frac)
        counts = int(np.clip(us / LEGACY_US_PER_COUNT, LEGACY_COUNT_MIN, LEGACY_COUNT_MAX))
        self.pwm.set_pwm(s.ch, 0, counts)

    def pwm_off(self, name):
        self.pwm.set_pwm(self.by_name[name].ch, 0, 0)

    # calibrated grasp targets (circuitpython path)
    def finger_limits(self, ch):
        return self.servos[int(ch)].limits

    def validate(self, channels=None) -> bool:
        ok = True
        for ch in channels if channels is not None else sorted(self.calib.channels, key=int):
            s = self.servos[int(ch)]
            lo, hi = s.limits
            for key in ("position", "release"):
                val = getattr(s, key)
                if not lo <= val <= hi:
                    print(f"  ! {s.name}: {key}={val} is outside travel {lo}-{hi}")
                    ok = False
        if ok:
            print("  calibration OK")
        return ok

    def move_finger(self, ch, target_us):
        lo, hi = self.finger_limits(ch)
        pos = int(round(max(lo, min(hi, target_us))))
        if abs(pos - target_us) > 1:
            print(f"  clamped {self.servos[int(ch)].name}: {target_us:.0f} -> {pos}")
        self.move_us(int(ch), pos)

    def set_fingers(self, key, wrist_now=None, channels=None):
        """key is 'position' (grasp) or 'release'. Applies tenodesis if wrist_now given."""
        for ch in channels if channels is not None else sorted(self.calib.channels, key=int):
            target = getattr(self.servos[int(ch)], key)
            if wrist_now is not None:
                target += self.calib.tenodesis_k * (wrist_now - self.calib.wrist_tilt_ref)
            self.move_finger(ch, target)

    def sim_to_real(self, ch, angle):
        angle_flexed, angle_straight = EXTRA_FINGERS[ch]["flexed"], EXTRA_FINGERS[ch]["straight"]
        angle = (angle - angle_straight)/(angle_flexed - angle_straight)
        us = FingerBoard.frac_to_us(np.clip(angle, 0, 1))
        return us

# =============================================================================
# Real robot
# =============================================================================

class RobotArm:
    """ST bus servos + PCA9685 fingers. Nothing opens until connect()."""

    def __init__(self, environment: Literal['mac', 'linux'] = 'linux', calib: Optional[Calibration] = None,
                 finger_backend: Literal["circuitpython", "legacy"] = "circuitpython"):
        self.environment = environment
        self.calib = calib if calib is not None else Calibration.load()
        self.bus = STBus()
        self.fingers = FingerBoard(self.calib, freq=self.calib.pwm_freq, backend=finger_backend)
        self.wrist_rotator = None
        self.wrist_tilter = None
        self.elbow = None

    def connect(self, st: bool = True, fingers: bool = True):
        if st:
            port = STBus.find_port(self.environment)
            if not self.bus.connect(port, self.calib.st_baud):
                raise RuntimeError(f"Failed to open {port} at {self.calib.st_baud} baud")
            self.wrist_rotator = self.bus.servo("wrist rotate")
            self.wrist_tilter = self.bus.servo("wrist tilt")
            self.elbow = self.bus.servo("elbow")
        if fingers:
            self.fingers.connect()
        return self

    def st(self, servo_id: int) -> STServo:
        return {s.id: s for s in (self.wrist_rotator, self.wrist_tilter, self.elbow)}[servo_id]

    def wrist_to(self, pos):
        self.st(self.calib.wrist_id).move(int(pos), self.calib.wrist_speed, self.calib.wrist_accel)
        return int(pos)

    def close(self):
        self.bus.close()


# =============================================================================
# Hand pose (human keypoints -> angles)
# =============================================================================

class HandPose:
    """Angles from 21-point hand keypoints (WiLoR / OpenPose order)."""

    @staticmethod
    def bone_vector(p1, p2):
        p1, p2 = np.array(p1), np.array(p2)
        x1, y1, z1 = p1
        x2, y2, z2 = p2
        vect = [(x1 - x2), (y1 - y2), (z1 - z2)]
        return vect

    @staticmethod
    def finger_angle(mcp, pip, tip) -> float:
        """Angle at pip between the mcp->pip and pip->tip bones, in degrees."""
        m = HandPose.bone_vector(mcp, pip)
        t = HandPose.bone_vector(pip, tip)
        dot_product = np.dot(m, t)
        m_len = np.linalg.norm(m)
        t_len = np.linalg.norm(t)

        cos_theta = dot_product / (m_len * t_len)
        cos_theta = np.clip(cos_theta, -1.0, 1.0)

        angle_radians = np.arccos(cos_theta)
        angle_degrees = np.degrees(angle_radians)
        return angle_degrees

    @staticmethod
    def finger_index(finger: int) -> tuple:
        mcp = 1 + (4 * finger)
        return mcp, mcp + 1, mcp + 3

    @staticmethod
    def apply_angles_arr(frame):
        angles = np.full(6, np.nan)
        wrist = frame[0]

        for f in range(5):
            mcp, pip, tip = HandPose.finger_index(f)
            total_curl_val = (HandPose.finger_angle(wrist, frame[mcp], frame[pip])
                              + HandPose.finger_angle(frame[mcp], frame[pip], frame[tip]))
            angles[f] = total_curl_val
        angles[5] = HandPose.finger_angle(wrist, frame[2], frame[5])
        return angles

    @staticmethod
    def palm_frame(mcp_i, mcp_p, mcp_m, w):
        """(normal, x_axis, y_axis). x = y x normal, matched to the MuJoCo hand."""
        y_axis = np.array(HandPose.bone_vector(mcp_m, w))
        x_axis = np.array(HandPose.bone_vector(mcp_i, mcp_p))
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        normal = np.cross(x_axis, y_axis)
        x_axis = np.cross(y_axis, normal)
        return normal, x_axis, y_axis

    @staticmethod
    def palm_frame_of(frame):
        return HandPose.palm_frame(frame[5], frame[17], frame[9], frame[0])

    @staticmethod
    def tilting(yv0, yv, normal):
        trans_x = np.dot(yv, yv0)
        trans_y = np.dot(normal, yv)
        t = np.degrees(np.arctan2(trans_y, trans_x))
        return t

    @staticmethod
    def rolling(xv, normal0, normal):
        trans_x = np.dot(normal0, xv)
        trans_y = np.dot(normal0, normal)
        t = np.degrees(np.arctan2(trans_y, trans_x))
        return t


# =============================================================================
# Kinematics
# =============================================================================

class ReferenceHandFK:
    """Forward kinematics over a hand-measured LINKS table.

    This is an independent model used to validate the MJCF; MjcArm is the source
    of truth. The table itself lives in main_motion/f_kinematics.py.
    """

    def __init__(self, links: list):
        self.links = links
        self.by_name = {link["name"]: link for link in links}

    @classmethod
    def default(cls):
        from main_motion.f_kinematics import LINKS
        return cls(LINKS)

    # rodrigues' rotation formula
    def rodrig(self, joint: str, angle: float):
        K = np.zeros((3, 3))
        joint = self.by_name[joint]
        axis = joint["axis"]
        K[0][1] = -(axis[2])
        K[0][2] = axis[1]
        K[1][0] = axis[2]
        K[1][2] = -(axis[0])
        K[2][0] = -(axis[1])
        K[2][1] = axis[0]
        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
        return R

    def transform(self, joint: str, angle: float):
        output = np.eye(4)
        joint = self.by_name[joint]
        axis = joint["axis"]
        if axis is not None:
            output[:3, :3] = self.rodrig(joint["name"], angle)
        output[:3, 3] = joint["offset"]
        return output

    def find_parent(self, name):
        link = self.by_name[name]
        parent = link["parent"]
        if parent is None:
            return [link]
        return self.find_parent(parent) + [link]

    def fkine(self, joint, angles: dict):
        """Pose of one joint; angles maps every link name on the chain to an angle."""
        M = np.eye(4)
        for j in self.find_parent(joint):
            name = j["name"]
            M = M @ self.transform(name, angles[name])
        return M

    def fkine_all(self, q: dict):
        pose = {}
        for link in self.links:
            driver = link["driver"]
            if driver is None:
                angle = q.get(link["name"], 0.0)
            else:
                angle = link["ratio"] * q.get(driver, 0.0)
            b = link["parent"]
            joint = link["name"]
            if b is None:
                pose[joint] = self.transform(joint, angle)
            else:
                if b in pose:
                    x = pose[b] @ self.transform(joint, angle)
                    pose[joint] = x
        return pose

    def expansion(self, q):
        """Leader angles -> every joint, applying the coupling ratios."""
        Q = {}
        for link in self.links:
            if link["driver"] is None:
                val = q.get(link["name"], 0.0)
                Q[link["name"]] = val
            else:
                val = q[link["driver"]]
                Q[link["name"]] = val * link["ratio"]
        return Q


class ArmIK:
    """Planar elbow + wrist IK: wrist position from an MCP target (mm, degrees)."""

    def __init__(self, upper: float, lower: float):
        self.upper = upper    # elbow -> wrist, mm
        self.lower = lower    # wrist -> mcp, mm

    def solve(self, mcp):
        mcp = np.array(mcp)
        xm, ym, zm = mcp
        vect_AD = np.linalg.norm(mcp)
        rxy = np.hypot(xm, ym)

        cos_psi = (vect_AD**2 + self.upper**2 - self.lower**2) / (2 * self.upper * vect_AD)
        if abs(cos_psi) > 1:
                    return None
        psi = np.arccos(cos_psi)
        phi = np.arctan2(xm, ym)
        elbow_tilt = np.degrees(phi - psi)

        cos_f = (self.upper**2 + self.lower**2 - vect_AD**2) / (2 * self.upper * self.lower)
        if abs(cos_f) > 1:
            return None
        cos_f = np.arccos(cos_f)
        wrist_tilt_a = 180 - np.degrees(cos_f)

        C = (self.upper * np.sin(np.radians(elbow_tilt)), self.upper * np.cos(np.radians(elbow_tilt)), 0)

        return wrist_tilt_a, elbow_tilt, mcp, C


# =============================================================================
# MuJoCo simulation
# =============================================================================

class MjcArm:
    """Wrapper around model/palm_with_frame.xml.

    Geometry, joint ranges and coupling ratios are always read from the loaded
    model, never duplicated here.
    """

    def __init__(self, xml=XML_PATH):
        import mujoco
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.renderer = None

    # id lookups
    def joint_id(self, name) -> int:
        return self.model.joint(name).id

    def actuator_id(self, name) -> int:
        return self.model.actuator(name).id

    def body_id(self, name) -> int:
        return self.model.body(name).id

    def site_id(self, name) -> int:
        try:
            return self.model.site(name).id
        except KeyError:
            raise KeyError(f"no site {name!r} in the model - add a <site> to {XML_PATH.name}") from None

    # model-derived values
    def joint_range(self, name) -> tuple:
        lo, hi = self.model.jnt_range[self.joint_id(name)]
        return float(lo), float(hi)

    def ctrl_range(self, act) -> tuple:
        lo, hi = self.model.actuator_ctrlrange[self.actuator_id(act)]
        return float(lo), float(hi)

    def couplings(self) -> dict:
        """follower joint -> (leader joint, polycoef[5]) from the <equality> block."""
        out = {}
        for i in range(self.model.neq):
            if self.model.eq_type[i] != self.mj.mjtEq.mjEQ_JOINT:
                continue
            follower = self.model.joint(self.model.eq_obj1id[i]).name
            leader = self.model.joint(self.model.eq_obj2id[i]).name
            out[follower] = (leader, self.model.eq_data[i][:5].copy())
        return out

    # state
    def reset(self):
        self.mj.mj_resetData(self.model, self.data)

    def qpos_adr(self, joint) -> int:
        return self.model.jnt_qposadr[self.joint_id(joint)]

    def set_leader(self, joint, q, data=None):
        """Set a leader joint's qpos and every coupled follower through its polycoef."""
        d = self.data if data is None else data
        d.qpos[self.qpos_adr(joint)] = q
        for follower, (leader, poly) in self.couplings().items():
            if leader != joint:
                continue
            x = q - self.model.qpos0[self.qpos_adr(leader)]
            y = sum(c * x**k for k, c in enumerate(poly))
            d.qpos[self.qpos_adr(follower)] = self.model.qpos0[self.qpos_adr(follower)] + y

    def set_ctrl(self, act, value, clip=False):
        i = self.actuator_id(act)
        if clip:
            lo, hi = self.model.actuator_ctrlrange[i]
            value = float(np.clip(value, lo, hi))
        self.data.ctrl[i] = value

    def kinematics(self, data=None):
        self.mj.mj_kinematics(self.model, self.data if data is None else data)

    def step(self, n: int = 1):
        for _ in range(n):
            self.mj.mj_step(self.model, self.data)

    # readouts
    def site_pos(self, name, data=None):
        d = self.data if data is None else data
        return d.site_xpos[self.site_id(name)].copy()

    def body_pos(self, name, data=None):
        d = self.data if data is None else data
        return d.xpos[self.body_id(name)].copy()

    def world_to_wrist(self, p, data=None, wrist_body="wrist"):
        """World point -> coordinates in the wrist body frame."""
        d = self.data if data is None else data
        b = self.body_id(wrist_body)
        R = d.xmat[b].reshape(3, 3)
        return R.T @ (np.asarray(p) - d.xpos[b])

    # rendering
    def make_renderer(self):
        self.renderer = self.mj.Renderer(self.model)
        return self.renderer

    def camera(self, distance=0.7, azimuth=70, elevation=-20, lookat=(0, 0, 0.5)):
        cam = self.mj.MjvCamera()
        cam.distance, cam.azimuth, cam.elevation, cam.lookat = distance, azimuth, elevation, list(lookat)
        return cam

    def vis_option(self, show_joints: bool):
        opt = self.mj.MjvOption()
        opt.flags[self.mj.mjtVisFlag.mjVIS_JOINT] = show_joints
        return opt

    def render(self, cam, opt):
        self.renderer.update_scene(self.data, cam, opt)
        return self.renderer.render()

    

# =============================================================================
# Camera
# =============================================================================

class Camera:
    """OpenCV capture. Nothing opens until open()."""

    def __init__(self, source=0):
        self.source = source
        self.cap = None

    @staticmethod
    def gstreamer_pipeline(sensor_id=0, capture_width=1920, capture_height=1080,
                           display_width=960, display_height=540, framerate=30, flip_method=0):
        """Only applies if a Jetson is connected to a CSI camera."""
        return f"nvarguscamerasrc sensor-id={sensor_id} ! video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, framerate={framerate}/1 ! nvvidconv flip-method={flip_method} ! video/x-raw, width={display_width}, height={display_height} ! appsink"

    def open(self):
        import cv2
        self.cap = cv2.VideoCapture(self.source)
        return self

    def is_open(self) -> bool:
        return self.cap is not None and self.cap.isOpened()

    def read(self):
        return self.cap.read()

    def release(self):
        if self.cap is not None:
            self.cap.release()

    @staticmethod
    def find_checkerboard(frame, pattern=(9, 6)):
        import cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        return cv2.findChessboardCorners(gray, pattern, None)


if __name__ == "__main__":
    arm = RobotArm('linux')
    arm.connect()
    arm.wrist_rotator.move()


# =============================================================================
# Footage -> arm motion
# =============================================================================

class DepthScale:
    """Turns WiLoR output from the overhead tripod footage into arm motion.

    Placeholders until the Thursday refilm:
        F_REAL_PX  - real focal in px at 960 wide: lens_mm / 22.3 * 960 (600D, APS-C)
        FWD_CAM    - image direction Hania's forearm points along the table
        ELBOW_REST - MuJoCo elbow angle (deg) with the forearm horizontal
    """

    F_REAL_PX = 940.0                         # ≈ 22 mm on the 600D kit lens
    UP_CAM = np.array([0.0, 0.0, -1.0])       # towards the camera = up from the table
    FWD_CAM = np.array([0.0, -1.0, 0.0])      # placeholder: forearm points up the image
    ELBOW_REST = 0.0
    FOREARM_MM = 324.3 
    FWD_OFFSET = 0.0      # mm, placeholder
    UP_OFFSET = 320.0     # mm, placeholder                       # elbow -> wrist tilt (LINKS: 174.3 + 150)

    @staticmethod
    def cam_tr(camera_translation, fl):
        """Correct WiLoR's camera_translation for the real focal length.

        Only z depends on the focal, so x and y are left as they are.
        Works on one frame (3,) or a whole take (n, 3). Returns a copy, in metres.
        """
        t = np.array(camera_translation, dtype=float)
        t[..., 2] *= DepthScale.F_REAL_PX / fl
        return t

        
    @staticmethod
    def ik_targets(mcp_cam):
        """ArmIK targets (n, 3) in mm from the MCP trajectory in camera coords (n, 3), metres.
        +y = up (ArmIK zero = model zero), +x = forward, z = 0 (no sideways joint)."""
        mcp_cam = np.asarray(mcp_cam, dtype=float)
        rel = (mcp_cam - np.nanmedian(mcp_cam, axis=0)) * 1000
        fwd = rel @ DepthScale.FWD_CAM + DepthScale.FWD_OFFSET
        up = rel @ DepthScale.UP_CAM + DepthScale.UP_OFFSET
        return np.column_stack([fwd, up, np.zeros(len(rel))])

    @staticmethod
    def fill_gaps(x):
        """Linearly interpolate NaN frames along time. Returns (filled copy, mask of frames that were NaN)."""
        x = np.array(x, dtype=float)
        flat = x.reshape(len(x), -1)
        missing = np.isnan(flat).any(axis=1)
        idx = np.arange(len(flat))
        for c in range(flat.shape[1]):
            flat[missing, c] = np.interp(idx[missing], idx[~missing], flat[~missing, c])
        return flat.reshape(x.shape), missing

    @staticmethod
    def smooth(x, median=5, window=11, poly=2):
        """Remove single-frame spikes (median), then jitter (Savitzky-Golay), along time.

        Works on (n,) or (n, k). Dropped frames are filled for the filters,
        then set back to NaN so the loop still skips them.
        """
        from scipy.ndimage import median_filter
        from scipy.signal import savgol_filter
        x, missing = DepthScale.fill_gaps(x)
        x = median_filter(x, size=(median,) + (1,) * (x.ndim - 1))
        x = savgol_filter(x, window_length=window, polyorder=poly, axis=0)
        x[missing] = np.nan
        return x