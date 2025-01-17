import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        # YOLO模型加载
        self.model = YOLO("/home/aruco/vision-ws/src/aruco-ros/yolo_pkg/weights/best.pt")

        
        self.color_sub = self.create_subscription(Image,'/realsense/cam/color/image_raw',
            self.color_callback,
            10
        )

        self.bbox_pub = self.create_publisher(Image, '/detected/bounding_boxes', 10)

        self.bridge = CvBridge()

        self.color_msg = None

        self.get_logger().info("VisionNode initialized and ready.")

    def color_callback(self, msg):
        self.color_msg = msg

        self.detect_objects()

    def detect_objects(self):
        if self.color_msg is None:
            self.get_logger().warning("No image message received yet.")
            return

        
        cv_image = self.bridge.imgmsg_to_cv2(self.color_msg, desired_encoding='bgr8')

        
        results = self.model(cv_image)

        for object in results:
            
            boxes = object.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  
                confidence = box.conf[0].item()
                label = box.cls[0].item() 

                self.get_logger().info(
                    f"Detected object: x1={x1}, y1={y1}, x2={x2}, y2={y2}, "
                    f"confidence={confidence:.2f}, label={label}"
                )

        results_img = results[0].plot()  
        self.bbox_pub.publish(self.bridge.cv2_to_imgmsg(results_img, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    vision_node = VisionNode()

    try:
        rclpy.spin(vision_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
