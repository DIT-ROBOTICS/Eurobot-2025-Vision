import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import object_tracking_ros as ob

class RealSenseListener(Node):
    def __init__(self):
        super().__init__('realsense_listener')
        
        self.color_subscription = self.create_subscription(
            Image,
            '/realsense/camera/color/image_raw',
            # self.color_callback,
            self.track_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            "/realsense/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10
        )

        self.color_image = None
        self.color_contour = None
        self.depth_image = None
        self.depth_contour = None

        self.tracker = cv2.TrackerCSRT_create()
        self.tracking = [False]

        self.multitracker = cv2.legacy.MultiTracker_create()
        self.multitracking = [False]
        
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

    def track_callback(self, msg):
        try:
            color_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            color_normalized_image = cv2.normalize(color_cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
            self.color_image = color_normalized_image

            # ob.Track(self.color_image, self.tracker, self.tracking)
            # ob.MultiTrack(self.color_image, self.multitracker, self.multitracking)
            bbox = ob.ColorRecTrans('src/object-tracking-ros/image/first_color_frame.png')
            print(bbox)
            ob.AutoTrack(self.color_image, self.tracker, self.tracking, bbox)

        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

def main(args=None):
    rclpy.init(args=args)

    realsense_listener = RealSenseListener()

    while rclpy.ok():
        rclpy.spin_once(realsense_listener)

    realsense_listener.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()