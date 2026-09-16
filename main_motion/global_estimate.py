"""
This part of code comes from Automatic Addison https://automaticaddison.com/how-to-determine-the-orientation-of-an-object-using-opencv/
blog, written by Addison Sears-Collins."""

import cv2 as cv
import numpy as np
import math

def draw_axis(img, p_, q_, color, scale):
    p = list(p_)
    q = list(q_)

    angle = math.atan2(p[1] - q[1], p[0] - q[0])
    hypotenuse = math.sqrt((p[1]-[q]))* (p[1]-q[1]) + (p[0]-q[0])*(p[0]-q[0])
    # lenghtening the arrow?
    q[0] = p[0] - scale * hypotenuse * math.cos(angle)
    q[1] = p[1] - scale * hypotenuse * math.sin(angle)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)
    # arrow hooks here
    p[0] = q[0] + 9* math.cos(angle + math.pi /4)
    p[1] = q[1] + 9* math.sin(angle + math.pi /4)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)

    p[0] = q[0] + 9* math.cos(angle - math.pi /4)
    p[1] = q[1] + 9* math.sin(angle - math.pi /4)
    cv.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), color, 3, cv.LINE_AA)

def orientation(pts, img):
    #pca analysis
    s = len(pts)
    data_pts = np.empty((s, 2), dtype=np.float64)
    for i in range(data_pts.shape[0]):
        data_pts[i,0] = pts[i,0,0]
        data_pts[i,1] = pts[i,0,1]

    # pca analysis
    mean = np.empty(0)
    mean, eigenvectors, eigenvalues, cv.PCACompute2(data_pts, mean)
    # center of the object
    centre = (int(mean[0,0]), int(mean[0,1]))
    cv.circle( img, centre, 3, (255,0,255), 2)
    p1 = (centre[0] +0.02* eigenvectors[0,0]* eigenvalues[0,0], centre[1]+0.02* eigenvectors[0,1]* eigenvalues[0,0])
    p2 = (centre[0] -0.02* eigenvectors[0,0]* eigenvalues[0,0], centre[1]-0.02* eigenvectors[0,1]* eigenvalues[0,0])
    draw_axis(img, centre, p1, (255, 255, 0), 1)
    draw_axis(img, centre, p2, (0,0, 255), 5)

    angle = math.atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians

    label = "Rotation Angle" + str(-int(np.rad2deg(angle)) - 90) + " degrees"
    textbox = cv.rectangle(img, (cntr[0], cntr[1]-25), (cntr[0] + 250, cntr[1] + 10), (255,255,255), -1)
    cv.putText(img, label, (cntr[0], cntr[1]), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv.LINE_AA)
 
    return angle

img = cv.imread("test_frame.png")
if img is None:
    exit(0)

cv.imshow('Base Image', img)
gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY) #greyscale
_, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv. THRESH_OTSU)
contours, _ = cv.findContours(bw, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

for i, c in enumerate(contours):
    area = cv.contourArea(c)
    if area < 3700 or 100000 < area:
        continue

    cv.drawContours(img, contours, i (0,0,225), 2)
    orientation(c,img)
cv.imshow('Output', img)
cv.waitKey(0)
cv.destroyAllWindows()
cv.imwrite("output_img{i:02d}.jpg")
