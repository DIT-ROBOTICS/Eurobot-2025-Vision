#!/usr/bin/env python3

import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int8MultiArray
from yolo.srv import RD_signal
from cv_bridge import CvBridge
from threading import Thread, Event

# ROS2 uses a different method to import YOLO
from ultralytics import YOLO

# Define variables
WIN_WIDTH, WIN_HEIGHT = 848, 480  # camera resolution
WEIGHT_PATH = "vision-ws/src/ultralytics-ros/weight/best.pt"
VERBOSE = False  # YOLO verbose (showing detection output)
WAITING_TIME = 0  # waiting time for first publish

six_region_map = {
    # "Region": [x1, x2, y1, y2]
    "1": [-0.625, -0.375, -0.425, -0.175],
    "2": [-0.625, -0.375, 0.175, 0.425],
    "3": [-0.125, 0.125, -0.625, -0.375],
    "4": [-0.125, 0.125, 0.375, 0.625],
    "5": [0.375, 0.625, -0.425, -0.175],
    "6": [0.375, 0.625, 0.175, 0.425]
}

class VisionNode(Node):
    def __init__(self):
        super().__init__('CB_Server')

        # YOLO model
        self.model = YOLO(WEIGHT_PATH)
        self.results_img = None

        # Publisher
        self.pub = self.create_publisher(Int8MultiArray, '/robot/objects/global_info', 10)
        self.six_plant_info = Int8MultiArray()
        self.six_plant_info.data = [0] * 6
        self.yolo_result_pub = self.create_publisher(Image, '/robot/objects/yolo_result', 10)
        self.camera_point_pub = self.create_publisher(PointStamped, '/robot/objects/camera_point', 10)

        self.camera_point = PointStamped()
        self.camera_point.header.frame_id = "realsense_camera"
        self.camera_point.header.stamp = self.get_clock().now().to_msg()

        # Subscriber
        self.col1_msg = None
        self.dep1_msg = None
        self.create_subscription(Image, "/cam1/color/image_raw", self.col_callback1, 10)
        self.create_subscription(Image, "/cam1/aligned_depth_to_color/image_raw", self.dep_callback1, 10)

        # Ready signal service
        self.srv = self.create_service(RD_signal, '/robot/startup/ready_signal', self.ready_callback)

        # Other tools
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info("CB waiting for Ready Signal...")
        self.publish_thread = None
        self.first_publish_event = Event()

    def col_callback1(self, msg):
        self.col1_msg = msg

    def dep_callback1(self, msg):
        self.dep1_msg = msg

    def ready_callback(self, request, response):
        ready = request.ready
        if ready:
            self.get_logger().info("Received ready signal. Starting CB detection...")
            if self.publish_thread is None or not self.publish_thread.is_alive():
                self.first_publish_event.clear()

                # YOLO STARTS HERE
                self.publish_thread = Thread(target=self.yolo)
                self.publish_thread.start()

                # Wait for the first publish to complete
                self.first_publish_event.wait()

            response.success = True
        else:
            self.get_logger().warn("Received ready signal is False. Aborting process...")
            response.success = False

        return response

    def preprocess(self, col1_msg: Image, dep1_msg: Image) -> np.ndarray:
        # Convert col_msg
        cv_col1_img = self.bridge.imgmsg_to_cv2(col1_msg, desired_encoding="bgr8")
        np_col1_img = np.asarray(cv_col1_img, dtype=np.uint8)

        # Convert dep_msg
        cv_dep1_img = self.bridge.imgmsg_to_cv2(dep1_msg, desired_encoding="passthrough")
        np_dep1_img = np.asarray(cv_dep1_img, dtype=np.uint16)

        return np_col1_img, np_dep1_img

    def yolo(self):
        while rclpy.ok():
            if self.col1_msg is not None and self.dep1_msg is not None:
                color_img, depth_img = self.preprocess(self.col1_msg, self.dep1_msg)

                # YOLO detection
                results = self.model(source=color_img, verbose=VERBOSE)

                for object in results:
                    self.results_img = object.plot()
                    self.yolo_result_pub.publish(self.bridge.cv2_to_imgmsg(self.results_img, encoding="bgr8"))

                    boxes = object.boxes
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        pixel_x, pixel_y = round((x1 + x2) / 2), round((y1 + y2) / 2)
                        depth = depth_img[pixel_y, pixel_x]
                        # Transform coordinates
                        world_x, world_y = self.transform_coordinates(pixel_x, pixel_y, depth)
                        # Check 6 plant info
                        self.six_plant_info_check(world_x, world_y)

                # From first publish check
                if not self.first_publish_event.is_set():
                    self.get_clock().sleep_for(rclpy.duration.Duration(seconds=WAITING_TIME))
                    self.get_logger().info("Processing YOLO...")
                    self.first_publish_event.set()

                # Publish 6 plant info
                self.pub.publish(self.six_plant_info)
                self.six_plant_info.data = [0] * 6

    def transform_coordinates(self, x, y, depth):
        self.camera_point.point.x = (depth * (x - 436.413) / 604.357) / 1000
        self.camera_point.point.y = (depth * (y - 245.459) / 604.063) / 1000
        self.camera_point.point.z = depth / 1000
        self.camera_point.header.stamp = self.get_clock().now().to_msg()

        try:
            self.tf_buffer.can_transform('map', 'realsense_camera', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
            transform = self.tf_buffer.lookup_transform('map', 'realsense_camera', rclpy.time.Time())
            world_point = tf2_geometry_msgs.do_transform_point(self.camera_point, transform)
            self.camera_point_pub.publish(world_point)
            return world_point.point.x, world_point.point.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            self.get_logger().error(f"Transform error: {ex}")
            return None

    def six_plant_info_check(self, x, y):
        # If match the region, set the corresponding value to 1
        for region, value in six_region_map.items():
            if value[0] < x < value[1] and value[2] < y < value[3]:
                self.six_plant_info.data[int(region) - 1] = 1

def main(args=None):
    rclpy.init(args=args)
    try:
        vision_node = VisionNode()
        rclpy.spin(vision_node)
    except rclpy.exceptions.ROSInterruptException:
        pass
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
