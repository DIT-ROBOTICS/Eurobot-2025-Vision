from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Int16, Int32
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
            '/vision/aruco/sima4/average_pose',
            self.superstar_callback,
            10
        )

        self.create_subscription(
            PoseArray,
            '/vision/aruco/sima_poseArray',
            self.sima_callback,
            10
        )

        self.create_subscription(
            Int16,
            '/vision/aruco/onboard/layer',
            self.layer_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/vision/aruco/robot/average_pose',
            self.robot_callback,
            10
        )

        self.create_subscription(
            PoseArray,
            '/detected/global_center_poses/platform',
            self.platform_callback,
            10
        )

        self.create_subscription(
            Int32,
            '/main/mission/success',
            self.mision_callback,
            10
        )

    def superstar_callback(self, msg: PoseStamped):
        with self.lock:
            self.last_superstar_pose_ = msg

    def sima_callback(self, msg: PoseArray):
        with self.lock:
            self.last_sima_pose_array_ = msg

    def layer_callback(self, msg: Int16):
        with self.lock:
            self.last_layer_ = msg

    def robot_callback(self, msg: PoseStamped):
        with self.lock:
            self.last_robot_pose_ = msg

    def platform_callback(self, msg: PoseArray):
        with self.lock:
            self.last_platform_pose_array_ = msg

    def mision_callback(self, msg: Int32):
        with self.lock:
            self.last_mission_ = msg

    def get_sensor_data(self):
        with self.lock:
            return {
                'superstar_pose': self.last_superstar_pose_,
                'sima_pose_array': self.last_sima_pose_array_,
                'layer': self.last_layer_,
                'robot_pose': self.last_robot_pose_,
                'platform_pose_array': self.last_platform_pose_array_
            }
