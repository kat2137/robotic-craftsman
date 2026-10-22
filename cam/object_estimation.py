'''This code was copied from a Computer Vision Tutorial by Pysource on YouTube'''

import cv2
import numpy as np

VIDEO_PATH = 'new_imitation.mp4'
FRAME_NO = 0

points = []
def draw_circle(event, x, y, flags, params):
    global points
    if event ==cv2.EVENT_LBUTTONDOWN:
        points.append((x,y))

def main():

    cv2.namedWindow("Frame")
    cv2.setMouseCallback("Frame", draw_circle)
    cap = cv2.VideoCapture(VIDEO_PATH)
    cap.set(cv2.CAP_PROP_POS_FRAMES, FRAME_NO)   # a frame where the stick is sharp
    _, img = cap.read()
    h, w = img.shape[:2]
    img = cv2.resize(img, (960, round(h * 960 / w)))

    # needs physical calibration -> adding a meter into the capture frame
    while True:
        frame = img.copy()
        for i, pt in enumerate(points):
            cv2.circle(frame, pt, 5, (25,15,255))
            cv2. putText(frame, str(i * 10), (pt[0] + 6, pt[1] - 6),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (25, 15, 255), 1)
            
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1)
    
        if key == 27:
            break
        elif key == ord("u") and points:
            points.pop()

    cap.release()
    cv2.destroyAllWindows()
    np.save("stick_points.npy", np.array(points))
    print(len(points), "points, image centre:", img.shape[1] / 2, img.shape[0] / 2)
    


if __name__ == "__main__":
    main()
