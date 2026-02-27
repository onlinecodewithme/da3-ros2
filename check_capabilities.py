import cv2
import sys

def check_camera(index):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print(f"Index {index}: Cannot open")
        return False
    
    formats = [
        (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), "MJPG"),
        (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'), "YUYV")
    ]
    
    print(f"Index {index}: Opened successfully")
    for prop, val, name in formats:
        cap.set(prop, val)
        actual = cap.get(prop)
        print(f"  {name}: Requested {val}, Got {actual} ({'Match' if val == actual else 'Mismatch'})")
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ret, frame = cap.read()
        if ret:
            print(f"  {name}: Capture success {frame.shape}")
        else:
            print(f"  {name}: Capture failed")
            
    cap.release()
    return True

if __name__ == "__main__":
    for i in range(10):
        check_camera(i)
