import cv2
import time
cap = cv2.VideoCapture(0)
time.sleep(2)
for i in range(30):
    ret, frame = cap.read()
cap.release()
gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
found, corners = cv2.findChessboardCorners(gray, (9,6), None)
print (f"Found {corners}")
cv2.imwrite("check.jpg", frame)
