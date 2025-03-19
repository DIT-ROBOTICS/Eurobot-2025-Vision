from time import sleep
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

output_dir = '/home/realsense/vision-ws/images/'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

class CameraMerger(Node):
    def __init__(self):
        super().__init__('camera_logger')

        self.bridge = CvBridge()
        self.camera_sub = self.create_subscription(
            Image,
            '/realsense2/cam_mid/color/image_raw',
            self.camera_callback,
            10
        )

        self.camera_image = None
        self.images_saved = False
        self.done = False

    def camera_callback(self, msg):
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.save_images()

    def save_images(self):
        if self.camera_image is not None:
            # Save images if not already saved
            if not self.images_saved:
                cv2.imwrite(os.path.join(output_dir, 'image2.jpg'), self.camera_image)
                self.images_saved = True

        if self.images_saved:
            self.get_logger().info('Images saved successfully. Shutting down...')
            self.done = True

def main(args=None):
    rclpy.init(args=args)
    camera_merger = CameraMerger()
    camera_merger.get_logger().info('Camera logger node started')

    while rclpy.ok() and not camera_merger.done:
        rclpy.spin_once(camera_merger, timeout_sec=0.1)

    camera_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
