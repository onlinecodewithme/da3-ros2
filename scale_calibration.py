#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class ScaleCalibrationNode(Node):
    def __init__(self):
        super().__init__('scale_calibration_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw', # Use raw image for texture
            self.image_callback,
            10)
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_anything_v3/depth',
            self.depth_callback,
            10)
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera_info',
            self.info_callback,
            10)
            
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_depth = None
        self.points = []
        self.scale_factor = 1.0
        # Default Intrinsics (will be overwritten)
        self.fx = 500.0
        self.fy = 500.0
        self.cx = 320.0
        self.cy = 240.0
        self.got_intrinsics = False

        self.get_logger().info("Scale Calibration Node Started. Click 2 points on the image window to measure distance.")

    def info_callback(self, msg):
        if not self.got_intrinsics:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.got_intrinsics = True
            self.get_logger().info(f"Got Intrinsics: fx={self.fx}, fy={self.fy}, cx={self.cx}, cy={self.cy}")

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.display_image() # Only update display on new RGB frame
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def depth_callback(self, msg):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f"Error converting depth: {e}")

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.latest_depth is None:
                self.get_logger().warn("No depth data received yet!")
                return

            depth_val = self.latest_depth[y, x]
            self.points.append((x, y, depth_val))
            self.get_logger().info(f"Point {len(self.points)}: ({x}, {y}) Depth: {depth_val:.3f}m (approx)")

            if len(self.points) == 2:
                self.calculate_distance()
                self.points = [] # Reset

    def calculate_distance(self):
        p1 = self.points[0]
        p2 = self.points[1]

        z1 = p1[2]
        x1 = (p1[0] - self.cx) * z1 / self.fx
        y1 = (p1[1] - self.cy) * z1 / self.fy

        z2 = p2[2]
        x2 = (p2[0] - self.cx) * z2 / self.fx
        y2 = (p2[1] - self.cy) * z2 / self.fy
        
        # Euclidean distance in 3D
        dist = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

        print(f"\n[MEASUREMENT]")
        print(f"  Point A: {p1[0]},{p1[1]} (Depth: {z1:.3f}m)")
        print(f"  Point B: {p2[0]},{p2[1]} (Depth: {z2:.3f}m)")
        print(f"  VIRTUAL DISTANCE: {dist:.4f} meters")
        print("-" * 30)
        print("To calibrate:")
        print(f"  Scale Factor = (REAL_DISTANCE) / {dist:.4f}")
        print("-" * 30)

    def display_image(self):
        if self.latest_image is not None:
            display = self.latest_image.copy()
            for p in self.points:
                cv2.circle(display, (p[0], p[1]), 5, (0, 0, 255), -1)
            
            cv2.imshow("Scale Calibration", display)
            cv2.setMouseCallback("Scale Calibration", self.click_event)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ScaleCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
