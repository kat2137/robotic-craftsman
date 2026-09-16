import numpy as np
import time
import sys

sys.path.append("..")

from scservo_sdk import * 
from Adafruit_PCA9685 import PCA9685
from angle_computation import apply_angles_arr, bone_vector
from servo_translate_new import move, SERVOS, ORDER
from st_motors_setup.tuning_st_servos import move_st, read

COUNTS = 11.28
SPEED = 300
ACCEL = 30
# wrist, elbow and shoulder operators are wired to a separate bus servo control board
pwm = PCA9685(busnum=1)
pwm.set_pwm_freq(60)

# Initialize PortHandler instance and set the port path for st servos
portHandler = PortHandler('/dev/ttyACM0')
# Initialize PacketHandler instance, get methods and members of Protocol
packetHandler = sms_sts(portHandler)  
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baud rate 1000000
if portHandler.setBaudRate(1000000):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

def palm_frame(mcp_i, mcp_p, mcp_m, w):
    y_axis = np.array(bone_vector(mcp_m, w))
    x_axis = np.array(bone_vector(mcp_i, mcp_p))
    y_axis = y_axis/np.linalg.norm(y_axis)
    x_axis = x_axis/np.linalg.norm(x_axis)
    normal = np.cross(x_axis,y_axis)
    x_axis = np.cross(normal, y_axis)
    return normal, x_axis, y_axis

def tilt(yv0, yv, normal):
    trans_x = np.dot(yv, yv0)
    trans_y = np.dot(normal, yv)
    t = np.degrees(np.arctan2(trans_y, trans_x))
    return t

def roll(xv, normal0, normal):
    trans_x = np.dot(normal0, xv)
    trans_y = np.dot(normal0, normal)
    t = np.degrees(np.arctan2(trans_y, trans_x))
    return t

def step(data):
    joints = data["joints"]
    angles = np.array([apply_angles_arr(i) for i in range(len(joints))])
    normal0, xv0, yv0 = palm_frame(joints[0][5], joints[0][17], joints[0][9], joints[0][0])
    init_tilt = read(1)/COUNTS
    init_roll= read(2)/COUNTS
    lo = np.nanpercentile(angles, 5, axis=0)
    hi = np.nanpercentile(angles, 95, axis=0)
    
    for row in angles:
        try:
            for i, name in enumerate(ORDER):
                move(name, row[i], lo[i], hi[i])
                print(f"{name}: {row[i]:.1f} ({lo[i]:.1f}, {hi[i]:.1f})")
                normal, xv, yv = palm_frame(joints[row][5], joints[row][17], joints[row][9], joints[row][0])
                tilt = tilt(yv0, yv, normal0)
                roll = roll(xv, normal0, normal)

                move_st(1, ((init_roll + roll)*COUNTS), SPEED, ACCEL)
                move_st(2, ((init_tilt + tilt)*COUNTS), SPEED, ACCEL)
                time.sleep(4/30)
        finally:
            for i, name in enumerate(ORDER):
                move(name, lo[i], lo[i], hi[i])
                time.sleep(0.5)
            for name in ORDER:
                pwm.set_pwm(SERVOS[name]["ch"], 0, 0)
         
def main():
    data = np.load("handsewing_01.npz")