import threading
import cv2
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import queue

class MultiCamNode(Node):
    def __init__(self):
        super().__init__('camera_merger')
        
        self.bridge = CvBridge()
        self.callback_group = ReentrantCallbackGroup()

        self.camera_subscribers = {
            "camera_1": {
                "topic": '/realsense1/cam_left/color/image_raw',
                "queue": queue.Queue(),
                "thread": None,
            },
            "camera_2": {
                "topic": '/realsense2/cam_mid/color/image_raw',
                "queue": queue.Queue(),
                "thread": None,
            },
            "camera_3": {
                "topic": '/realsense3/cam_right/color/image_raw',
                "queue": queue.Queue(),
                "thread": None,
            },
        }

        for camera, info in self.camera_subscribers.items():
            self.create_subscription(
                Image,
                info["topic"],
                lambda msg, cam=camera: self.image_callback(cam, msg),
                10,
                callback_group=self.callback_group
            )
            
            info["thread"] = threading.Thread(
                target=self.process_images, args=(camera,), daemon=True
            )
            info["thread"].start()

        # 添加 Publisher
        self.publisher = self.create_publisher(Image, '/stitched_image', 10)

    def image_callback(self, camera, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        self.camera_subscribers[camera]["queue"].put(image)

    def process_images(self, camera):
        while True:
            try:
                image = self.camera_subscribers[camera]["queue"].get(timeout=1)
                self.get_logger().info(f"Processed image from {camera}")
            except queue.Empty:
                continue

    def publish_stitched_image(self, images):
        stitched_image = np.hstack(images)
        stitched_image = cv2.resize(stitched_image, (1280, 720))
        msg = self.bridge.cv2_to_imgmsg(stitched_image, encoding='bgr8')
        self.publisher.publish(msg)
