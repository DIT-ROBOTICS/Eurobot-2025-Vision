import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import object_tracking_ros as ob

class RealSenseListener(Node):
    def __init__(self):
        super().__init__('realsense_listener')
        
        self.color_subscription = self.create_subscription(
            Image,
            '/realsense/cam/color/image_raw',
            # self.color_callback,
            self.track_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            "/realsense/cam/depth/image_rect_raw",
            self.depth_callback,
            10
        )

        self.color_image = None
        self.color_contour = None
        self.depth_image = None
        self.depth_contour = None

        self.tracker = cv2.TrackerCSRT_create()
        self.tracking = False
        
        self.bridge = CvBridge()

    def color_callback(self, msg):
        try:
            color_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            color_normalized_image = cv2.normalize(color_cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
            self.color_image = color_normalized_image
            print(f"dtype{color_cv_image.dtype}")

            # ob.Track(self.color_image, self.tracker, self.tracking)

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

    def track_callback(self, msg):
        try:
            color_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            color_normalized_image = cv2.normalize(color_cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
            self.color_image = color_normalized_image

            if self.tracking == False:
                area = cv2.selectROI('Select', self.color_image, False, False)
                self.tracker.init(self.color_image, area)
                self.tracking = True
            
            if self.tracking == True:
                success, point = self.tracker.update(self.color_image)
                if success:
                    p1 = [int(point[0]), int(point[1])]
                    p2 = [int(point[0] + point[2]), int(point[1] + point[3])]
                    cv2.rectangle(self.color_image, p1, p2, (0,0,255), 3)

            cv2.imshow('Track', self.color_image)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)

    realsense_listener = RealSenseListener()

    # realsense_listener.tracker, realsense_listener.tracking = ob.Select(realsense_listener.color_image, realsense_listener.tracker, realsense_listener.tracking)

    while rclpy.ok():
        rclpy.spin_once(realsense_listener)

    realsense_listener.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()