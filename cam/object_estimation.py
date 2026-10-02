'''This code was copied from a Computer Vision Tutorial by Pysource on YouTube'''

import cv2
import math

from arm import Camera

#same units measured in the frame
PIXELS = 0
CM = 0

points = []
def draw_circle(event, x, y, flags, params):
    global points
    if event ==cv2.EVENT_LBUTTONDOWN:
        if len(points) ==2:
            points = []
        points.append((x,y))

def main():
    RATIO = CM/PIXELS
    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", draw_circle)
    cap = Camera(0).open()


    # needs physical calibration -> adding a meter into the capture frame
    while True:
        _, frame = cap.read()
        for pt in points:
            cv2.circle(frame, pt, 5, (25,15,255))
        if len(points) == 2:
            vect_px = math.hypot[(points[0][0] - points[1][0], points[1][0] - points[1][1])]
            distance_cm = vect_px * RATIO

            #rect = cv2.minAreaRect(segment)
            #box = cv2.boxPoints(rect)
            #center, size, angle = rect
            #size1 = round(size[0]*RATIO, 2)
            #size2 = round(size[1]*RATIO, 2)
        

            cv2. putText(frame, f"{vect_px}")
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)

        if key == 27:
            break


if __name__ == "__main__":
    main()
