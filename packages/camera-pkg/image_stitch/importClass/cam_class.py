from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import cv2
import numpy as np
import message_filters

class MultiCamNode(Node):
    def __init__(self):
        super().__init__('multi_cam_node')
        self.bridge = CvBridge()
        
        self.fps = 0
        self.image_queue = []
        self.log_time = time.time()

        self._declare_parameters()
        self.publisher_qos = self._create_qos_profile()
        self._init_endpoints()

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_mid, self.sub_right], queue_size=10, slop=0.05,
        )
        self.ts.registerCallback(self.image_callback)

    def image_callback(self, msg_left, msg_mid, msg_right):
        if msg_left.encoding != msg_mid.encoding or msg_mid.encoding != msg_right.encoding:
            self.get_logger().warn("Encodings from cameras do not match. This may cause unexpected behavior.")
        if self.source_image_shape is None or self.data_type is None or self.encoding is None:
            self.encoding = msg_mid.encoding
            self._encoding_trans(self.encoding, msg_mid.height, msg_mid.width)

        image_left = self.preprocess_image_data(msg_left.data)
        image_mid = self.preprocess_image_data(msg_mid.data)
        image_right = self.preprocess_image_data(msg_right.data)
        stacked_images = np.stack([image_mid, image_left, image_right])
        if len(self.image_queue) >= 10:
            self.image_queue.pop(0)
        self.image_queue.append(stacked_images)

    def preprocess_image_data(self, data):
        try:
            image = np.frombuffer(data, dtype=self.data_type).reshape(self.source_image_shape)
        except ValueError as e:
            self.get_logger().error(f"Reshape failed: {e}")
            return np.zeros((*self.target_image_shape, 3), dtype=np.uint8)
        image = cv2.resize(image, self.target_image_shape)
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
        # save one image to disk 
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)
        self.terminal_log()

    def terminal_log(self):
        current_time = time.time()
        elapsed_time = current_time - self.log_time 
        self.fps += 1

        if elapsed_time >= 1.0:
            avg_fps = self.fps / elapsed_time
            self.get_logger().info(f"{'-' * 40}")
            self.get_logger().info(f"Publishing stitched image")
            self.get_logger().info(f"Average FPS: {avg_fps:.2f}")
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
                ('target_image_shape', '360x640'),
            ]
        )

        image_shape_mapping = {
            '360x640': (640, 360),
            '480x848': (848, 480),
            '720x1280': (1280, 720),
            '1080x1920': (1920, 1080),
        }

        self.encoding = None
        self.data_type = None
        self.source_image_shape = None
        self.target_image_shape = image_shape_mapping.get(self.get_parameter('target_image_shape').get_parameter_value().string_value, (360, 640))

    def _encoding_trans(self, encoding, height, width):
        encoding = encoding.lower()

        shape_mapping = {
            'bgr8': (height, width, 3),
            'rgb8': (height, width, 3),
            'mono8': (height, width),
            '16uc1': (height, width),
            '32uc1': (height, width),
        }

        dtype_mapping = {
            'bgr8': np.uint8,
            'rgb8': np.uint8,
            'mono8': np.uint8,
            '16uc1': np.uint16,
            '32fc1': np.float32,
        }

        if encoding in shape_mapping:
            self.source_image_shape = shape_mapping.get(encoding, (360, 640, 3))
            self.data_type = dtype_mapping.get(encoding, np.uint8)
        else:
            self.get_logger().error(f"Unsupported encoding: {encoding}")
            raise ValueError(f"Unsupported encoding: {encoding}")
        
    def _init_endpoints(self):
        self.publisher = self.create_publisher(Image, 
                                               self.get_parameter('stitched_topic').get_parameter_value().string_value, 
                                               self.publisher_qos)

        self.sub_left = message_filters.Subscriber(self, Image,
                                                   self.get_parameter('left_topic').get_parameter_value().string_value,
                                                   qos_profile=self._create_qos_profile())
        self.sub_mid = message_filters.Subscriber(self, Image,
                                                  self.get_parameter('mid_topic').get_parameter_value().string_value,
                                                  qos_profile=self._create_qos_profile())
        self.sub_right = message_filters.Subscriber(self, Image,
                                                    self.get_parameter('right_topic').get_parameter_value().string_value,
                                                    qos_profile=self._create_qos_profile())
        
    def _create_qos_profile(self):
        return QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )
