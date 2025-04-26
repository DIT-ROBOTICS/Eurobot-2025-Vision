import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32MultiArray
import math

class RegionDetector(Node):
    def __init__(self):
        super().__init__('region_detector')

        self.pose_subscriber = self.create_subscription(PoseArray, 'detected/global_center_poses/platform', self.pose_callback, 10)
        self.region_publisher = self.create_publisher(Int32MultiArray, '/detected/global_center_poses/has_material', 10)

        self.material_points = [(2.18, 1.275),(2.925,1.32),(2.925,0.395),(1.895,0.95),(2.22,0.25),(0.78,0.25),
        (1.095,0.95),(0.075,0.395),(0.075,1.32),(0.82,1.725)]
            

    def pose_callback(self, msg):
        region_flags = [0] * len(self.material_points)  
        for pose in msg.poses:
            x, y, z = pose.position.x, pose.position.y, pose.position.z
            for i, (point_x,point_y) in enumerate(self.material_points):
                self.get_logger().info(f'Checking distance between ({x},{y}) and ({point_x},{point_y})')
                if (self.get_distance(x, y, point_x, point_y) < 0.15 and z <0.05):
                    region_flags[i] = 1

        region_msg = Int32MultiArray()
        region_msg.data = region_flags
        self.region_publisher.publish(region_msg)
        self.get_logger().info(f'Published Region Flags: {region_flags}')
    
    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def main(args=None):
    rclpy.init(args=args)
    node = RegionDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
