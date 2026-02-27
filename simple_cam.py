#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2

class SimpleCam(Node):
    def __init__(self):
        super().__init__('usb_cam')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.info_publisher_ = self.create_publisher(CameraInfo, 'camera_info', 10)
        self.timer = self.create_timer(0.033, self.timer_callback) # 30 FPS
        self.cap = cv2.VideoCapture(0)
        if self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.get_logger().info('SimpleCam node started with MJPG 640x480 on device 0')
            self.bridge = CvBridge()
        else:
            self.get_logger().error('Could not open video device 0')
            self.cap = None

    def timer_callback(self):
        if self.cap is None:
            return
        
        # Retry logic for frame reading
        ret = False
        for _ in range(5):
             ret, frame = self.cap.read()
             if ret:
                 break
        
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link_optical"
            self.publisher_.publish(msg)
            
            # Publish dummy camera info for synchronization
            info_msg = CameraInfo()
            info_msg.header = msg.header
            info_msg.width = frame.shape[1]
            info_msg.height = frame.shape[0]
            
            # Approximate intrinsics for 640x480
            fx = 480.0
            fy = 480.0
            cx = 320.0
            cy = 240.0
            
            info_msg.k = [fx, 0.0, cx, 
                          0.0, fy, cy, 
                          0.0, 0.0, 1.0]
            info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            info_msg.p = [fx, 0.0, cx, 0.0,
                          0.0, fy, cy, 0.0,
                          0.0, 0.0, 1.0, 0.0]
                          
            self.info_publisher_.publish(info_msg)
        else:
            self.get_logger().warn('Failed to capture frame')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleCam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
