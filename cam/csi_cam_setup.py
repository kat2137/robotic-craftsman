import cv2
from arm import Camera

#gstreamer only applies if Jetson is connected to a csi camera
gstreamer_pipeline = Camera.gstreamer_pipeline

def display(n=1):
    title = "webcam"
    video_on = Camera(0).open()
    if video_on.is_open():
        try:
            for i in range (n):
                ret, frame = video_on.read()
                if not ret:
                    print("read failed")
                    break
                cv2.imwrite("frame{i:03d}.jpg", frame)
                print("Saved frame{i:03d}.jpg")
        finally:
            video_on.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    keyCode = cv2.waitKey(10) & 0xFF
    display()
