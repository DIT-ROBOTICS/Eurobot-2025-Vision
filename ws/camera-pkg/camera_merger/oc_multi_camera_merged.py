import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraMerger(Node):
    def __init__(self):
        super().__init__('camera_merger')

        self.bridge = CvBridge()
        self.camera_1_sub = self.create_subscription(
            Image,
            '/realsense1/cam_left/color/image_raw',
            self.camera_1_callback,
            10
        )
        self.camera_2_sub = self.create_subscription(
            Image,
            '/realsense2/cam_mid/color/image_raw',
            self.camera_2_callback,
            10
        )
        self.camera_3_sub = self.create_subscription(
            Image,
            '/realsense3/cam_right/color/image_raw',
            self.camera_3_callback,
            10
        )

        self.camera_1_image = None
        self.camera_2_image = None
        self.camera_3_image = None

        self.merged_image_pub = self.create_publisher(
            Image, '/realsense_merged/merged_image', 10  
        )

    def camera_1_callback(self, msg):
        self.camera_1_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.save_images()

    def camera_2_callback(self, msg):
        self.camera_2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.save_images()

    def camera_3_callback(self, msg):
        self.camera_3_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.save_images()

    def save_images(self):
        cv2.imwrite('~/vision-ws/images/camera_1_image.png', self.camera_1_image)
        cv2.imwrite('~/vision-ws/images/camera_2_image.png', self.camera_2_image)
        cv2.imwrite('~/vision-ws/images/camera_3_image.png', self.camera_3_image)
    
def main(args=None):
    rclpy.init(args=args)
    camera_merger = CameraMerger()
    rclpy.spin(camera_merger)
    camera_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
