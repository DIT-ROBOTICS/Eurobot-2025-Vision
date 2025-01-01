import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# 計算透視變換矩陣 M
M = cv2.getPerspectiveTransform(src_pts, dst_pts)
H_left = np.array([[1.0, 0.0, 0.0],
                     [0.0, 1.0, 0.0],
                     [0.0, 0.0, 1.0]])
H_right = np.array([[1.0, 0.0, 0.0],
                        [0.0, 1.0, 0.0],
                        [0.0, 0.0, 1.0]])

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
        self.camera_1_image = self.transform_image(self.camera_1_image, H_left)
        self.merge_images()

    def camera_2_callback(self, msg):
        self.camera_2_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.merge_images()

    def camera_3_callback(self, msg):
        self.camera_3_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.camera_3_image = self.transform_image(self.camera_3_image, H_right)
        self.merge_images()
    
    def transform_image(self, image, H):
        # 應用透視變換
        return cv2.warpPerspective(image, H, (image.shape[1], image.shape[0]))

    def merge_images(self):
        if self.camera_1_image is not None and self.camera_2_image is not None and self.camera_3_image is not None:

            # 垂直拼接影像
            merged_image = np.vstack((self.camera_1_image, self.camera_2_image, self.camera_3_image))
            
            self.merge_image_publisher(merged_image)

    def merge_image_publisher(self, merged_image):
        merged_image_msg = self.bridge.cv2_to_imgmsg(merged_image, encoding='bgr8')
        self.merged_image_pub.publish(merged_image_msg)

def main(args=None):
    rclpy.init(args=args)
    camera_merger = CameraMerger()
    rclpy.spin(camera_merger)
    camera_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
