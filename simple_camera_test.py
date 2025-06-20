import cv2

def test_camera():
    # Try different backends
    backends = [
        cv2.CAP_DSHOW,  # DirectShow
        cv2.CAP_MSMF,   # Media Foundation
        cv2.CAP_ANY     # Any available backend
    ]
    
    for backend in backends:
        print(f"\nTrying backend: {backend}")
        cap = cv2.VideoCapture(0, backend)
        
        if not cap.isOpened():
            print(f"Failed to open camera with backend {backend}")
            continue
        
        print(f"Camera opened successfully with backend {backend}!")
        print("Trying to capture a frame...")
        
        ret, frame = cap.read()
        if ret:
            print("Successfully captured a frame!")
            print(f"Frame shape: {frame.shape}")
            cap.release()
            return
        else:
            print("Failed to capture frame")
        
        cap.release()
    
    print("\nCould not open camera with any backend")

if __name__ == "__main__":
    test_camera() 