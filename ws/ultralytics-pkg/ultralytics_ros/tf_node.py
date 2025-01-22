import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
    
class TFNode(Node):
    def __init__(self):
        super().__init__('tf_node')

        # TF 緩衝器和監聽器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 訂閱中心點座標
        self.center_sub = self.create_subscription(PointStamped, '/detected/cam_center_points', self.center_callback, 10)

        # 發布全局座標
        self.global_pub = self.create_publisher(PointStamped, '/detected/global_center_points', 10)
        self.get_logger().info("TF Node initialized and ready.")

    def center_callback(self, msg):
        try:
            # 轉換中心點座標到 map
            transformed_point = self.tf_buffer.transform(
                msg, target_frame="map", timeout=rclpy.duration.Duration(seconds=1.0)
            )

            self.get_logger().info(
                f"Global point: x={transformed_point.point.x}, y={transformed_point.point.y}, z={transformed_point.point.z}"
            )

            # 發布轉換後的結果
            self.global_pub.publish(transformed_point)

        except Exception as e:
            self.get_logger().error(f"Transform failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    tf_node = TFNode()

    try:
        rclpy.spin(tf_node)
    except KeyboardInterrupt:
        pass
    finally:
        tf_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
