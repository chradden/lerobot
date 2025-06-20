import cv2
import os

def test_camera():
    # Open the camera
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    
    print("Camera opened successfully!")
    
    # Read a frame
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Could not read frame")
        return
    
    # Create output directory if it doesn't exist
    os.makedirs('output', exist_ok=True)
    
    # Save the frame
    output_path = os.path.join('output', 'test_camera.jpg')
    cv2.imwrite(output_path, frame)
    print(f"Test image saved to: {output_path}")
    
    # Release the camera
    cap.release()

if __name__ == "__main__":
    test_camera() 