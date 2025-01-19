import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # YOLO模型
        self.model = YOLO("/home/ultralytics/vision-ws/src/ultralytics-ros/weight/best.pt")

        # 訂閱相機影像
        self.color_sub = self.create_subscription(
            Image,
            '/realsense/camera/color/image_raw',
            self.color_callback,
            10
        )
        self.bbox_pub = self.create_publisher(Image, '/detected/bounding_boxes', 10)
        
        self.color_msg = None
        # 發布檢測到的物件中心座標（相機座標系）
        self.center_pub = self.create_publisher(PointStamped, '/detected/cam_center_points', 10)

        # CvBridge
        self.bridge = CvBridge()

        self.get_logger().info("YOLO Node initialized and ready.")

    def color_callback(self, msg):
        self.color_msg = msg

        self.detect_objects()

    def detect_objects(self):
        if self.color_msg is None:
            self.get_logger().warning("No image message received yet.")
            return

        # 將影像訊息轉換為 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(self.color_msg, desired_encoding='bgr8')

        # 使用 YOLO 模型進行物件偵測
        results = self.model(cv_image)
    
        for object in results:
            boxes = object.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 邊界框座標
                confidence = box.conf[0].item()  # 置信度
                label = box.cls[0].item()  # 類別標籤

                # 計算邊界框中心座標
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                center_z = 0.0  # 深度相機

                # 創建相機框架的中心座標訊息
                center_point = PointStamped()
                center_point.header.frame_id = 'cam_mid_link'  # 相機的座標框架
                center_point.header.stamp = self.get_clock().now().to_msg()
                center_point.point.x = center_x
                center_point.point.y = center_y
                center_point.point.z = center_z

                self.get_logger().info(
                    f"Detected object: x1={x1}, y1={y1}, x2={x2}, y2={y2}, "
                    f"Centerpoints of camera coordinates: x={center_x:.2f}, y={center_y:.2f}, z={center_z:.2f}"
                    f"confidence={confidence:.2f}, label={label}"
                    
                )

                # 發布中心座標
                self.center_pub.publish(center_point)
        # 發布yolo辨識結果的影像
        results_img = results[0].plot()  
        self.bbox_pub.publish(self.bridge.cv2_to_imgmsg(results_img, encoding="bgr8"))


def main(args=None):
    rclpy.init(args=args)
    yolo_node = YoloNode()

    try:
        rclpy.spin(yolo_node)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
