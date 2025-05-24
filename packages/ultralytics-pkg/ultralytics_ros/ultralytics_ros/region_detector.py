import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32MultiArray
import math
class RegionDetector(Node):
    def __init__(self):
        super().__init__('region_detector')
        self.declare_parameter("platform_topic", "/vision/global_center_poses/platform")
        self.declare_parameter("set_topic", "/vision/global_center_poses/set")
        self.declare_parameter("has_material_topic", "/vision/global_center_poses/has_material")
        platform_topic = self.get_parameter("platform_topic").value
        has_material_topic = self.get_parameter("has_material_topic").value
        set_topic = self.get_parameter("set_topic").value
        self.material_points = [(2.18, 1.725), (2.925, 1.32), (2.925, 0.395), (1.895, 0.95), (2.22, 0.25), 
            (0.78, 0.25), (1.095, 0.95), (0.075, 0.395), (0.075, 1.32), (0.82, 1.725)]
        self.region_flags = [0] * len(self.material_points)
        self.platform_subscriber = self.create_subscription(PoseArray, platform_topic, self.platform_callback, 10)
        self.set_subscriber = self.create_subscription(PoseArray, set_topic, self.set_callback, 10)
        self.region_publisher = self.create_publisher(Int32MultiArray, has_material_topic, 10)
        self.timer = self.create_timer(1.0, self.print_region_flags) 
        self.set_pose = None
        self.platform_pose = None
    def set_callback(self, msg):
        self.set_pose = msg
    def platform_callback(self, msg):
        self.platform_pose = msg
        self.detect_material()
    def detect_material(self):
        if self.set_pose is not None and self.platform_pose is not None:
            self.region_flags = [0] * len(self.material_points)
            for pose in self.set_pose.poses:
                x, y, z = pose.position.x, pose.position.y, pose.position.z
                # print(f'Set Pose: x={x}, y={y}, z={z}')
                for i, (point_x, point_y) in enumerate(self.material_points):
                    if self.get_distance(x, y, point_x, point_y) < 0.20 :
                        self.region_flags[i] = 1
            for pose in self.platform_pose.poses:
                x, y, z = pose.position.x, pose.position.y, pose.position.z
                # print(f'Platform Pose: x={x}, y={y}, z={z}')
                for i, (point_x, point_y) in enumerate(self.material_points):
                    if (self.get_distance(x, y, point_x, point_y) < 0.20):
                        self.region_flags[i] = 1
            region_msg = Int32MultiArray()
            region_msg.data = self.region_flags  
            self.region_publisher.publish(region_msg)

    def get_distance(self, x1, y1, x2, y2):
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

    def print_region_flags(self):
        self.get_logger().info(f'Published Region Flags: {self.region_flags}')

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
