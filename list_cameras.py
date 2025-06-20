import cv2

def list_cameras():
    available_cameras = []
    for i in range(10):  # Check indices 0 to 9
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                available_cameras.append(i)
            cap.release()
    return available_cameras

if __name__ == "__main__":
    cameras = list_cameras()
    if cameras:
        print("Available cameras:", cameras)
    else:
        print("No cameras detected.") 