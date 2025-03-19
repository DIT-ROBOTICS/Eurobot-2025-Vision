from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import queue
import threading

class MultiCamNode(Node):
    def __init__(self):
        super().__init__('multi_cam_node')
        self.bridge = CvBridge()
        self.callback_group = ReentrantCallbackGroup()
        self.images = [None, None, None]
        self.img_queue = queue.Queue()
        self.fps = 0
        self.log_time = time.time()
        
        qos_profile = QoSProfile(depth=10, 
                                 reliability=QoSReliabilityPolicy.BEST_EFFORT, 
                                 history=QoSHistoryPolicy.KEEP_LAST,
                                 durability=QoSDurabilityPolicy.VOLATILE)
        
        self.publisher = self.create_publisher(Image, '/stitched_image', qos_profile)
        
        self.camera_subscribers = {
            "camera_1": {
                "topic": '/realsense1/cam_left/color/image_raw',
                "thread": None,
                "last_log_time": time.time(),
            },
            "camera_2": {
                "topic": '/realsense2/cam_mid/color/image_raw',
                "thread": None,
                "last_log_time": time.time(),
            },  
            "camera_3": {
                "topic": '/realsense3/cam_right/color/image_raw',
                "thread": None,
                "last_log_time": time.time(),
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

    def image_callback(self, camera, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cam_index = {"camera_1": 0, "camera_2": 1, "camera_3": 2}.get(camera)
        self.img_queue.put((cam_index, image))
        
    def process_images(self, camera):
        self.get_logger().info(f"Processing images for {camera}")
        while True:
            try:
                cam_index, image = self.img_queue.get(timeout=0.1)
                self.images[cam_index] = image
            except queue.Empty:
                continue   

    def get_images(self):
        if all(img is not None for img in self.images): 
            return self.images
        else:
            return None
    
    def publish_stitched_image(self, image):
        #stitched_image = cv2.resize(images, (917, 540))
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        #self.compressed_img_msg.data = cv2.imencode('.jpg', image)[1].tobytes()
        if msg is None:
            return
        self.terminal_log()
        self.publisher.publish(msg)

    def terminal_log(self):
        current_time = time.time()
        self.fps += 1

        if current_time - self.log_time >= 1:
            self.get_logger().info(f"{'-' * 40}")
            self.get_logger().info(f"Publishing stitched image")
            self.get_logger().info(f"Average FPS: {self.fps}")
            self.log_time = current_time
            self.fps = 0
