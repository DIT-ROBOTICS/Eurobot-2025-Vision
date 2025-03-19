import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO    
import numpy as np
import cv2
import time 


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # YOLO模型
        self.model = YOLO("/home/ultralytics/vision-ws/src/ultralytics-ros/weight/ver3_platform.pt")

        # 訂閱相機影像
        self.color_sub = self.create_subscription(Image,'/realsense/cam2/color/image_raw',
            self.color_callback,10)
        self.depth_sub = self.create_subscription(Image,'/realsense/cam2/depth/image_rect_raw', 
            self.depth_callback, 10)

        # 發布檢測到的物件中心座標（相機座標系）
        self.center_pub = self.create_publisher(PointStamped, '/detected/cam_center_points', 10)
        # CvBridge
        self.bridge = CvBridge()
        
        self.depth_image = None 
        self.color_msg = None

        # 用於 FPS 計算
        self.frame_count = 0
        self.start_time = time.time()

        self.get_logger().info("YOLO Node initialized and ready.")

    def depth_callback(self, msg):
        # 儲存最新的深度影像
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def color_callback(self, msg):

        self.color_msg = msg

        self.detect_objects()

        # 記錄開始時間（計算延遲）
        start_time = time.time()

        # # 記錄 ROS2 訊息時間戳記（轉換為秒）
        # ros_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # current_time = time.time()
        # latency = current_time - ros_timestamp  # 計算延遲
        
        # # 計算 FPS
        # self.frame_count += 1
        # elapsed_time = time.time() - self.start_time
        # fps=0.0
        # if elapsed_time >= 1.0:  # 每秒重置一次
        #     fps = self.frame_count / elapsed_time
        #     self.frame_count = 0
        #     self.start_time = time.time()
        # # 記錄處理時間
        # processing_time = time.time() - start_time
        # self.get_logger().info(f"FPS: {fps:.2f}, Latency: {latency:.4f} sec, Processing Time: {processing_time:.4f} sec")
        
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

                f_x = 609.68 #內參
                f_y = 608.42
                c_x = 425.38
                c_y = 238.43

                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 邊界框座標
                confidence = box.conf[0].item()  
                label = box.cls[0].item()  

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                point1 = self.switch_to_cam_coor(x1,y1)
                point2 = self.switch_to_cam_coor(x2,y2)
                center_point = self.switch_to_cam_coor(center_x,center_y)

                print(f"Detected object: x1={x1}, y1={y1}, x2={x2}, y2={y2}, \n")
                print(f"confidence={confidence:.2f}, label={label}\n")
                print(f"cam coordinate: x={center_point.point.x:.2f}, y={center_point.point.y:.2f}, z={center_point.point.z:.2f}\n")
                

                # 發布中心座標
                self.center_pub.publish(center_point)
        # 發布yolo辨識結果的影像
        results_img = results[0].plot()  

    def switch_to_cam_coor(self,x,y):
        f_x = 609.68 #內參
        f_y = 608.42
        c_x = 425.38
        c_y = 238.43

        if self.depth_image is not None:
            height, width = self.depth_image.shape
            z = self.depth_image[int(y), int(x)]
        else:
            z = 0.0
        point = PointStamped()
        point.header.frame_id = 'cam_mid_link'  # 相機的座標框架
        point.header.stamp = self.get_clock().now().to_msg()
        point.point.x = (z * (x - c_x) / f_x) / 1000 
        point.point.y = (z * (y - c_y) / f_y) / 1000
        point.point.z = z/1000
        return point

def main(args=None):
    rclpy.init(args=args)
    yolo_node_nopub = YoloNode()

    try:
        rclpy.spin(yolo_node_nopub)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_node_nopub.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
