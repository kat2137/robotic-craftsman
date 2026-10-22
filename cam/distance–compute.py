import numpy as np
import cv2
from scipy.optimize import least_squares
VIDEO_PATH = 'new_imitation.mp4'
FRAME_NO = 0
JSON = "imitation01.npz"


focal = 940 
Z = 200 #guessed distance

pts = np.load("stick_points.npy")[:-1].astype(float)   
s = np.arange(len(pts)) * 10.0       # mm along the stick, 1 cm apart
cap = cv2.VideoCapture(VIDEO_PATH)
_, img = cap.read()
h, w = img.shape[:2]
img = cv2.resize(img, (960, round(h * 960 / w)))
H, W = img.shape[:2]
cx, cy = W/2, H/2
ph0 = np.arctan2(pts[-1][1] - pts[0][1], pts[-1][0] - pts[0][0])
th0 = np.radians(45)
u, v = pts[0]
xbyz = (u - cx)/ focal
ybyz = (v - cy)/focal

X = xbyz*Z
Y = ybyz*Z

x0 = [focal, X, Y, Z, th0, ph0]

def residuals(x):
    f, X0, Y0, Z0, th, ph = x
    d = np.array([np.sin(th) * np.cos(ph),np.sin(th) * np.sin(ph),np.cos(th)])
    P = np.array([X0, Y0, Z0]) + s[:, None] * d
    
    u = f * P[:, 0] / P[:, 2] + cx
    v = f * P[:, 1] / P[:, 2] + cy
    projected = np.column_stack([u, v])
    return (projected - pts).ravel()

result = least_squares(residuals, np.array(x0))
print("F_REAL_PX:", result.x[0])
print("RMS px:", np.sqrt(np.mean(result.fun**2)))
print("Z0 mm:", result.x[3], " tilt deg:", np.degrees(result.x[4]))
print(np.round(result.fun.reshape(-1, 2), 1))
    
    
    