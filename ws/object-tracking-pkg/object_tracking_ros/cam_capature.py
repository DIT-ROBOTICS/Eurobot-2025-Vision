from rclpy.node import Node
from sensor_msgs.msg import Image
# from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np   


class RealSenseListener(Node):
    def __init__(self):
        super().__init__('realsense_listener')
        
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            "/camera/camera/depth/image_rect_raw",
            self.depth_callback,
            10
        )

        self.color_image = None
        self.color_contour = None
        self.depth_image = None
        self.depth_contour = None
        
        self.bridge = CvBridge()

    def color_callback(self, msg):
        try:
            color_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            color_normalized_image = cv2.normalize(color_cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
            self.color_image = color_normalized_image
            print(f"dtype{color_cv_image.dtype}")
            cv2.imshow("Color", self.color_image)
            cv2.waitKey(1)
            # self.color_contour = Color_Contour(self.color_image)

            self.get_logger().info(f"Received color image with shape: {self.color_image.shape}")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def depth_callback(self, msg):
        try:
            depth_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_normalized_image = cv2.normalize(depth_cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC3)
            self.depth_image = depth_normalized_image
            cv2.imshow("Depth", self.depth_image)
            cv2.waitKey(1)
            # self.depth_contour = Depth_Contour(self.depth_image)

            self.get_logger().info(f"Received depth image with shape: {self.depth_image.shape}")            
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")