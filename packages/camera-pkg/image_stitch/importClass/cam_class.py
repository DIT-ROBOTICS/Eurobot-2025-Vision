from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import numpy as np
import message_filters

class MultiCamNode(Node):
    def __init__(self):
        super().__init__('multi_cam_node')
        self.bridge = CvBridge()
        self.callback_group = ReentrantCallbackGroup()
        self.fps = 0
        self.image_queue = []
        self.log_time = time.time()

        self._declare_parameters()
        self.publisher_qos = self._create_qos_profile()
        self._init_endpoints()


        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_mid, self.sub_right], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.image_callback)

    def image_callback(self, msg_left, msg_mid, msg_right):
        image_left = self.rotate_image_data(msg_left.data)
        image_mid = self.rotate_image_data(msg_mid.data)
        image_right = self.rotate_image_data(msg_right.data)
        stacked_images = np.stack([image_mid, image_left, image_right])
        if len(self.image_queue) >= 10:
            self.image_queue.pop(0)
        self.image_queue.append(stacked_images)

    def rotate_image_data(self, data):
        image = np.frombuffer(data, dtype=self.data_type)
        image = image.reshape(self.image_shape)
        rotated_image = np.rot90(image) 
        return rotated_image

    def get_queue_images(self):
        if len(self.image_queue) == 0:
            return None
        return self.image_queue.pop(0)
        
    def publish_stitched_image(self, image):
        if image is None:
            return
        msg = self.bridge.cv2_to_imgmsg(image, encoding=self.encoding)
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.terminal_log()

    def terminal_log(self):
        current_time = time.time()
        self.fps += 1

        if current_time - self.log_time >= 1:
            self.get_logger().info(f"{'-' * 40}")
            self.get_logger().info(f"Publishing stitched image")
            self.get_logger().info(f"Average FPS: {self.fps}")
            self.log_time = current_time
            self.fps = 0
    
    def _declare_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[
                ('left_topic', '/realsense1/cam_left/color/image_raw'),
                ('mid_topic', '/realsense2/cam_mid/color/image_raw'),
                ('right_topic', '/realsense3/cam_right/color/image_raw'),
                ('stitched_topic', '/realsense/stitched_image/color/image_raw'),
                ('image_shape', '360x640x3'),
                ('encoding', 'bgr8'),
                ('data_type', 'uint8')
            ]
        )

        image_shape_mapping = {
            '360x640x3': (360, 640, 3),
            '360x640': (360, 640),
        }
        dtype_mapping = {
            'uint8': np.uint8,
            'uint16': np.uint16,
        }

        self.data_type = dtype_mapping.get(self.get_parameter('data_type').get_parameter_value().string_value, np.uint16)
        self.image_shape = image_shape_mapping.get(self.get_parameter('image_shape').get_parameter_value().string_value, (360, 640, 3))
        self.encoding = self.get_parameter('encoding').get_parameter_value().string_value

    def _init_endpoints(self):
        self.publisher = self.create_publisher(Image, 
                                               self.get_parameter('stitched_topic').get_parameter_value().string_value, 
                                               self.publisher_qos)

        self.sub_left = message_filters.Subscriber(self, Image,
                                                   self.get_parameter('left_topic').get_parameter_value().string_value)
        self.sub_mid = message_filters.Subscriber(self, Image,
                                                  self.get_parameter('mid_topic').get_parameter_value().string_value)
        self.sub_right = message_filters.Subscriber(self, Image,
                                                    self.get_parameter('right_topic').get_parameter_value().string_value)
        
    def _create_qos_profile(self):
        return QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )