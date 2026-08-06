import cv2
from cam.proxy_cam_live_rec import cap
cap = cv2.VideoCapture(0)


while True:
    cap()
