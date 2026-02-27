import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudInspector(Node):

    def __init__(self):
        super().__init__('point_cloud_inspector')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth_anything_v3/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.count = 0

    def listener_callback(self, msg):
        self.count += 1
        # if self.count % 5 != 0: 
        #    return
            
        print(f"Received PointCloud2 message: width={msg.width}, height={msg.height}", flush=True)
        
        # Read points
        points_list = []
        try:
            # Read only xyz
            gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False)
            points_list = list(gen)
            
            if not points_list:
                print("Empty point cloud!")
                return

            print(f"Sample point[0]: {points_list[0]} type: {type(points_list[0])}")
            
            # Manual stats calculation to avoid numpy casting issues
            total_points = len(points_list)
            num_nans = 0
            valid_points = []
            
            import math
            for p in points_list:
                # Assuming p is a tuple or indexable (x, y, z)
                x, y, z = p[0], p[1], p[2]
                if math.isnan(x) or math.isnan(y) or math.isnan(z):
                    num_nans += 1
                else:
                    valid_points.append((x, y, z))

            print(f"Total points: {total_points}")
            print(f"NaN points: {num_nans}")
            print(f"Valid points: {len(valid_points)}")

            if valid_points:
                valid_np = np.array(valid_points)
                min_vals = np.min(valid_np, axis=0)
                max_vals = np.max(valid_np, axis=0)
                avg_vals = np.mean(valid_np, axis=0)

                print(f"X range: {min_vals[0]:.4f} to {max_vals[0]:.4f}")
                print(f"Y range: {min_vals[1]:.4f} to {max_vals[1]:.4f}")
                print(f"Z range: {min_vals[2]:.4f} to {max_vals[2]:.4f}")
                print(f"Average: {avg_vals}")
            else:
                print("NO VALID POINTS!")
                
        except Exception as e:
            print(f"Error processing points: {e}")
            import traceback
            traceback.print_exc()
            return

def main(args=None):
    rclpy.init(args=args)
    inspector = PointCloudInspector()
    rclpy.spin(inspector)
    inspector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
