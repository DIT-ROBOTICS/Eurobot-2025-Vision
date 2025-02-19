import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseArray, Pose
from cv_bridge import CvBridge

import object_tracking_ros as ob

class RealSenseListener(Node):
    def __init__(self):
        super().__init__('realsense_listener')
        
        # Pubulisher
        self.publisher = self.create_publisher(Point, 'SIMA_POINT', 20)
        self

        # Publisher
        self.color_subscription = self.create_subscription(
            Image,
            '/realsense/camera/color/image_raw',
            # self.color_callback,
            self.color_track_callback,
            20
        )

        # self.depth_subscription = self.create_subscription(
        #     Image,
        #     "/realsense/camera/aligned_depth_to_color/image_raw",
        #     self.depth_track_callback,
        #     20
        # )

        # Color
        self.color_image = None
        self.color_contour = None

        self.color_tracker = cv2.TrackerCSRT_create()
        self.color_tracking = [False]

        self.color_multitracker = cv2.legacy.MultiTracker_create()
        self.color_multitracking = [False]

        # Depth
        # self.depth_image = None
        # self.depth_contour = None

        # self.depth_tracker = cv2.TrackerCSRT_create()
        # self.depth_tracking = [False]

        # self.depth_multitracker = cv2.legacy.MultiTracker_create()
        # self.depth_multitracking = [False]
        
        self.bridge = CvBridge()

    # def depth_track_callback(self, msg):
    #     try:
    #         self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    #         self.depth_image = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    #         # cv2.imshow('Depth_image', self.depth_image)
    #         # cv2.waitKey(1)
            
    #         # ob.Track(self.color_image, self.tracker, self.tracking)
    #         # ob.MultiTrack(self.color_image, self.multitracker, self.multitracking)
    #         # bbox = ob.DepthRecTrans('src/object-tracking-ros/image/first_depth_frame.png')
    #         # print(bbox)
    #         # self.depth_auto(self.depth_image, self.depth_tracker, self.depth_tracking, bbox)
          
    #     except Exception as e:
    #         self.get_logger().error(f"Error converting image: {e}")

    def color_track_callback(self, msg):
        try:
            # Process Image
            self.color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.color_image = cv2.normalize(self.color_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
            # cv2.imshow('Color_image', self.color_image)
            # cv2.waitKey(1)

            # ob.Track(self.color_image, self.color_tracker, self.color_tracking)
            # ob.MultiTrack(self.color_image, self.color_multitracker, self.color_multitracking)

            # Single SIMA
            # bbox = ob.ColorRecTrans('src/object-tracking-ros/image/first_color_frame.png')
            # p1, p2 = ob.AutoTrack(self.color_image, self.color_tracker, self.color_tracking, bbox)
            # middle = [(p1[0]+p2[0])/2, (p1[1]+p2[1])/2]
            # print(middle)

            # Four SIMA
            bbox_list = ob.ColorMultiRecTrans('src/object-tracking-ros/image/first_color_frame.png')
            sima_points = ob.AutoMultiTrack(self.color_image, self.color_multitracker, self. color_multitracking, bbox_list)

            # Publish
            SIMA_POINT = Point()
            SIMA_POINT.x = middle[0]
            SIMA_POINT.y = middle[1]

            self.publisher.publish(SIMA_POINT)
            self.get_logger().info(f"SIMA_POINT: {SIMA_POINT.x}, {SIMA_POINT.y}")


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