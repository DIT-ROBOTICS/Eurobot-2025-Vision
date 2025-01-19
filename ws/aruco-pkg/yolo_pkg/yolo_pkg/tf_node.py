import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener


class TfNode(Node):
    def __init__(self):
        super().__init__('tf_node')

        # 訂閱 YOLO 節點發布的中心座標（相機座標系）
        self.center_sub = self.create_subscription(
            PointStamped,
            '/detected/cam_center_points',
            self.center_callback,
            10
        )

        # 發布轉換後的全局座標
        self.global_pub = self.create_publisher(PointStamped, '/detected/global_center_points', 10)

        # TF Buffer 和 Listener
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("TF Node initialized and ready.")

    def center_callback(self, msg):
        try:
            # 將相機座標轉換為全局座標
            global_point = self.tf_buffer.transform(msg, 'world', timeout=rclpy.duration.Duration(seconds=1.0))
            self.get_logger().info(
                f"Global coordinates: x={global_point.point.x:.2f}, "
                f"y={global_point.point.y:.2f}, z={global_point.point.z:.2f}"
            )

            # 發布全局座標
            self.global_pub.publish(global_point)

        except Exception as e:
            self.get_logger().error(f"Transform failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    tf_node = TfNode()

    try:
        rclpy.spin(tf_node)
    except KeyboardInterrupt:
        pass
    finally:
        tf_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
