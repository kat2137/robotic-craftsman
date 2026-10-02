import cv2
from cam.proxy_cam_live_rec import cap


def main():
    cap = cv2.VideoCapture(0)

    while True:
        cap()


if __name__ == "__main__":
    main()
