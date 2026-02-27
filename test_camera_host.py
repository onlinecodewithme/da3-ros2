import cv2
import time

def test_camera():
    print("Attempting to open camera index 0...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Could not open video device 0")
        return

    # Set MJPG format
    print("Setting MJPG format...")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Allow camera to warm up
    time.sleep(2)

    print("Reading frame...")
    ret, frame = cap.read()
    
    if ret:
        print(f"Success! Captured frame with shape: {frame.shape}")
        # Optionally save the frame to verify visual output
        cv2.imwrite("host_camera_test.jpg", frame)
        print("Savings frame to host_camera_test.jpg")
    else:
        print("Error: Failed to capture frame")

    cap.release()
    print("Camera released.")

if __name__ == "__main__":
    test_camera()
