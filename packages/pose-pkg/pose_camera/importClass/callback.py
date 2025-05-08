from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Int16
import threading

class SensorCallback(Node):
    def __init__(self):
        super().__init__('sensor_callback_node')
        self.get_logger().info(f'Initializing {self.get_name()}...')
        self._init_state()
        self._init_subscriptions()
        self.get_logger().info(f'{self.get_name()} is up.')

    def _init_state(self):
        self.get_logger().info('Initializing state variables...')
        self.lock = threading.Lock()
        self.last_superstar_pose_ = None
        self.last_sima_pose_array_ = None
        self.last_layer_ = None
        self.last_robot_pose_ = None
        self.last_platform_pose_array_ = None

    def _init_subscriptions(self):
        self.get_logger().info('Creating subscriptions...')

        self.create_subscription(
            PoseStamped,
            '/vision/aruco/left/robot/pose',
            self.left_aruco_callback,
            10
        )

        self.create_subscription(
            PoseArray,
            '/vision/aruco/left/robot/pose',
            self.mid_aruco_callback,
            10
        )

        self.create_subscription(
            Int16,
            '/vision/aruco/left/robot/pose',
            self.right_aruco_callback,
            10
        )

    def left_aruco_callback(self, msg: PoseStamped):
        with self.lock:
            self.last_superstar_pose_ = msg

    def mid_aruco_callback(self, msg: PoseArray):
        with self.lock:
            self.last_sima_pose_array_ = msg

    def right_aruco_callback(self, msg: Int16):
        with self.lock:
            self.last_layer_ = msg

    def get_sensor_data(self):
        with self.lock:
            return {
                'superstar_pose': self.last_superstar_pose_,
                'sima_pose_array': self.last_sima_pose_array_,
                'layer': self.last_layer_,
                'robot_pose': self.last_robot_pose_,
                'platform_pose_array': self.last_platform_pose_array_
            }
