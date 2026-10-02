import cv2
import time
from pathlib import Path
from arm import Camera

HERE = Path(__file__).resolve().parent


def main():
    cap = Camera(0).open()
    time.sleep(2)
    for i in range(30):
        ret, frame = cap.read()
    cap.release()
    found, corners = Camera.find_checkerboard(frame, (9,6))
    print (f"Found {corners}")
    cv2.imwrite(str(HERE / "check.jpg"), frame)


if __name__ == "__main__":
    main()
