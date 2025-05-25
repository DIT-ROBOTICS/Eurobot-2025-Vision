import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import cv2
from ultralytics import YOLO
import torch
import threading
from sensor_msgs.msg import CompressedImage


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        qos_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.bridge = CvBridge()
        self.latest_image = None
        self.image_lock = threading.Lock()

        self.model = YOLO("/home/ultralytics/vision-ws/src/ultralytics-ros/weight/ver7_0507.pt")
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)

        self.subscription = self.create_subscription(
            Image,
            '/vision/stitched_image/color/image_raw',
            self.image_callback,
            qos_profile
        )

        self.publisher = self.create_publisher(
            CompressedImage,
            '/vision/stitched_image/color/image_annotated/compressed',
            qos_profile
        )

        self.timer = self.create_timer(0.08, self.timer_callback)  

        self.get_logger().info('Image subscriber node with YOLOv11 started (timer mode).')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.latest_image = (cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def timer_callback(self):
        with self.image_lock:
            if self.latest_image is None:
                return
            cv_image, header = self.latest_image

        results = self.model(cv_image)[0]
        annotated_image = cv_image.copy()
        for box in results.boxes:
            cls_id = int(box.cls[0])
            label = self.model.names[cls_id]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                annotated_image, f"{label} {conf:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2
            )

        try:
            success, jpeg_data = cv2.imencode('.jpg', annotated_image)
            if not success:
                self.get_logger().error("Failed to encode image to JPEG")
                return

            compressed_msg = CompressedImage()
            compressed_msg.header = header
            compressed_msg.format = "jpeg"
            compressed_msg.data = jpeg_data.tobytes()

            self.publisher.publish(compressed_msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing compressed image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
