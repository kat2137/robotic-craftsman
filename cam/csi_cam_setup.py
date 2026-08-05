import cv2

""" 
gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
Flip the image by setting the flip_method (most common values: 0 and 2)
display_width and display_height determine the size of each camera pane in the window on the screen
Default 1920x1080 displayd in a 1/4 size window
"""
#this function was a part of Nvidia Jetson CSI camera setup covered under MIT License.
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
def display():
    title = "CSI Camera"
    print(gstreamer_pipeline(flip_method=0))
    video_on = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
    if video_on.isOpened():
        window = cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
        while True:
            ret, frame = video_on.read()
            if not ret:
                break
            cv2.imshow(title, frame)
            if keyCode == 27:  # ESC key: quit program
                break
            elif keyCode == ord('s'):  # 's' key: save image
                cv2.imwrite("CSI_Camera_Image.jpg", frame)
                print("Saved CSI_Camera_Image.jpg")
