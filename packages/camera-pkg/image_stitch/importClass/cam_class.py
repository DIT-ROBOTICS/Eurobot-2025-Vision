from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
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

        pub_qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.publisher = self.create_publisher(Image, '/stitched_image', pub_qos_profile)

        self.sub_left = message_filters.Subscriber(self, Image, '/realsense1/cam_left/color/image_raw')
        self.sub_mid = message_filters.Subscriber(self, Image, '/realsense2/cam_mid/color/image_raw')
        self.sub_right = message_filters.Subscriber(self, Image, '/realsense3/cam_right/color/image_raw')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_mid, self.sub_right], queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.image_callback)

    def image_callback(self, msg_left, msg_mid, msg_right):
        image_left = self.rotate_image_data(msg_left.data)
        image_mid = self.rotate_image_data(msg_mid.data)
        image_right = self.rotate_image_data(msg_right.data)
        stacked_images = np.stack([image_mid, image_left, image_right])
        self.image_queue.append(stacked_images)

    def rotate_image_data(self, data):
        image = np.frombuffer(data, dtype=np.uint8)  
        image = image.reshape((360, 640, 3))  
        rotated_image = np.rot90(image) 
        return rotated_image

    def get_queue_images(self):
        if len(self.image_queue) == 0:
            return None
        return self.image_queue.pop(0)
        
    def publish_stitched_image(self, image):
        if image is None:
            return

        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        
        # msg.header.stamp = timestamp
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