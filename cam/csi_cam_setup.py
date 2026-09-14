import cv2
#gstreamer only applies if Jetson is connected to a csi camera
keyCode = cv2.waitKey(10) & 0xFF
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=960,
    display_height=540,
    framerate=30,
    flip_method=0,
):
    return f"nvarguscamerasrc sensor-id={sensor_id} ! video/x-raw(memory:NVMM), width={capture_width}, height={capture_height}, framerate={framerate}/1 ! nvvidconv flip-method={flip_method} ! video/x-raw, width={display_width}, height={display_height} ! appsink"
def display(n=1):
    title = "webcam"
    video_on = cv2.VideoCapture(0)
    if video_on.isOpened():
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
display()