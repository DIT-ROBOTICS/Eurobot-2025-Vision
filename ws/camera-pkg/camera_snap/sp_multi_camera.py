from time import sleep
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rclpy
import cv2
import os

output_dir = '/home/realsense/vision-ws/images/'
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

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
        self.images_saved = [False, False, False]
        self.done = False  # Flag to indicate when to stop spinning

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
        if all(img is not None for img in [self.camera_1_image, self.camera_2_image, self.camera_3_image]) and not all(self.images_saved):
            # Save images if not already saved
            if not self.images_saved[0]:
                self.camera_1_image = cv2.rotate(self.camera_1_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imwrite(os.path.join(output_dir, 'camera_1_image.jpg'), self.camera_1_image)
                self.images_saved[0] = True
            if not self.images_saved[1]:
                self.camera_2_image = cv2.rotate(self.camera_2_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imwrite(os.path.join(output_dir, 'camera_2_image.jpg'), self.camera_2_image)
                
                self.images_saved[1] = True
            if not self.images_saved[2]:
                self.camera_3_image = cv2.rotate(self.camera_3_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
                cv2.imwrite(os.path.join(output_dir, 'camera_3_image.jpg'), self.camera_3_image)
                
                self.images_saved[2] = True

        if all(self.images_saved):
            self.get_logger().info('Images saved successfully. Shutting down...')
            self.done = True  # Set the flag to stop spinning

def main(args=None):
    rclpy.init(args=args)
    camera_merger = CameraMerger()
    camera_merger.get_logger().info('Camera merger node started')

    # Loop to process messages until done
    while rclpy.ok() and not camera_merger.done:
        rclpy.spin_once(camera_merger, timeout_sec=0.1)

    camera_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
