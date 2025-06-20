import cv2

def list_cameras():
    """List all available cameras with detailed information."""
    print("Searching for cameras...")
    for i in range(10):  # Check first 10 indices
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                print(f"\nCamera #{i}:")
                print(f"  Resolution: {frame.shape[1]}x{frame.shape[0]}")
                print(f"  FPS: {cap.get(cv2.CAP_PROP_FPS)}")
                print(f"  Backend: {cap.getBackendName()}")
                print(f"  Format: {cap.get(cv2.CAP_PROP_FORMAT)}")
                print(f"  Mode: {cap.get(cv2.CAP_PROP_MODE)}")
                print(f"  Brightness: {cap.get(cv2.CAP_PROP_BRIGHTNESS)}")
                print(f"  Contrast: {cap.get(cv2.CAP_PROP_CONTRAST)}")
                print(f"  Saturation: {cap.get(cv2.CAP_PROP_SATURATION)}")
                print(f"  Exposure: {cap.get(cv2.CAP_PROP_EXPOSURE)}")
            cap.release()

if __name__ == "__main__":
    list_cameras() 