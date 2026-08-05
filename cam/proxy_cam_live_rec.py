import cv2
import os
import json
import visualize_pose as vp
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame.")
        break
    vp.main(frame)
    cv2.imshow('camera live', frame)
                  
