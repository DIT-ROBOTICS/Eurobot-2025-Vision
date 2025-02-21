# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import PointStamped
# from cv_bridge import CvBridge
# from ultralytics import YOLO    
# import numpy as np
# import cv2
# import time 


# class YoloNode(Node):
#     def __init__(self):
#         super().__init__('yolo_node')

#         # YOLO模型
#         self.model = YOLO("/home/ultralytics/vision-ws/src/ultralytics-ros/weight/ver4.pt")

#         # 訂閱相機影像
#         self.color_sub = self.create_subscription(Image,'/realsense2/cam_mid/color/image_raw',
#             self.color_callback,10)
#         self.depth_sub = self.create_subscription(Image,'/realsense2/cam_mid/aligned_depth_to_color/image_raw', 
#             self.depth_callback, 10)
#         self.bbox_pub = self.create_publisher(Image, '/detected/bounding_boxes', 10)
        
#         # 發布檢測到的物件中心座標（相機座標系）
#         self.center_pub = self.create_publisher(PointStamped, '/detected/cam_center_points', 10)
#         self.center_pub_platform = self.create_publisher(PointStamped, '/detected/cam_center_points/platform', 10)
#         self.center_pub_column = self.create_publisher(PointStamped, '/detected/cam_center_points/column', 10)
#         # CvBridge
#         self.bridge = CvBridge()
        
#         self.depth_image = None 
#         self.color_msg = None

#         # 用於 FPS 計算
#         self.frame_count = 0
#         self.start_time = time.time()

#         self.get_logger().info("YOLO Node initialized and ready.")

#     def depth_callback(self, msg):
#         # 儲存最新的深度影像
#         try:
#             self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         except Exception as e:
#             self.get_logger().error(f"Failed to process depth image: {e}")

#     def color_callback(self, msg):
#         self.color_msg = msg

#         self.detect_objects()

#         # # 記錄開始時間（計算延遲）
#         # start_time = time.time()

#         # # 記錄 ROS2 訊息時間戳記（轉換為秒）
#         # ros_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
#         # current_time = time.time()
#         # latency = current_time - ros_timestamp  # 計算延遲


#         # # 計算 FPS
#         # self.frame_count += 1
#         # elapsed_time = time.time() - self.start_time
#         # fps=0.0
#         # if elapsed_time >= 1.0:  # 每秒重置一次
#         #     fps = self.frame_count / elapsed_time
#         #     self.frame_count = 0
#         #     self.start_time = time.time()
#         # # 記錄處理時間
#         # processing_time = time.time() - start_time
#         # self.get_logger().info(f"FPS: {fps:.2f}, Latency: {latency:.4f} sec, Processing Time: {processing_time:.4f} sec")
        
#     def detect_objects(self):
#         if self.color_msg is None:
#             self.get_logger().warning("No image message received yet.")
#             return

#         # 將影像訊息轉換為 OpenCV 格式
#         cv_image = self.bridge.imgmsg_to_cv2(self.color_msg, desired_encoding='bgr8')

#         # 使用 YOLO 模型進行物件偵測
#         results = self.model(cv_image)
    
#         for object in results:
#             boxes = object.boxes
#             for box in boxes:

#                 f_x = 609.68 #內參
#                 f_y = 608.42
#                 c_x = 425.38
#                 c_y = 238.43

#                 x1, y1, x2, y2 = map(int, box.xyxy[0])  # 邊界框座標
#                 confidence = box.conf[0].item()  
#                 label = box.cls[0].item()  

#                 center_x = (x1 + x2) / 2
#                 center_y = (y1 + y2) / 2

#                 point1 = self.switch_to_cam_coor(x1,y1)
#                 point2 = self.switch_to_cam_coor(x2,y2)
#                 center_point = self.switch_to_cam_coor(center_x,center_y)

#                 print(f"Detected object: x1={x1}, y1={y1}, x2={x2}, y2={y2}, \n")
#                 print(f"confidence={confidence:.2f}, label={label}\n")
#                 print(f"cam coordinate: x={center_point.point.x:.2f}, y={center_point.point.y:.2f}, z={center_point.point.z:.2f}\n")

#                 # 發布中心座標
#                 self.center_pub.publish(center_point)
#                 if label == 0:
#                     self.center_pub_platform.publish(center_point)
#                 else:
#                     self.center_pub_column.publish(center_point)
#         # 發布yolo辨識結果的影像
#         results_img = results[0].plot()  
#         self.bbox_pub.publish(self.bridge.cv2_to_imgmsg(results_img, encoding="bgr8"))

#     def switch_to_cam_coor(self,x,y):
#         f_x = 609.68 #內參
#         f_y = 608.42
#         c_x = 425.38
#         c_y = 238.43

#         if self.depth_image is not None:
#             height, width = self.depth_image.shape
#             z = self.depth_image[int(y), int(x)]
#         else:
#             z = 2000
#         point = PointStamped()
#         point.header.frame_id = 'cam_mid_color_optical_frame'  # 相機的座標框架
#         point.header.stamp = self.get_clock().now().to_msg()
#         point.point.x = (z * (x - c_x) / f_x) / 1000 
#         point.point.y = (z * (y - c_y) / f_y) / 1000
#         point.point.z = z/1000
#         return point

# def main(args=None):
#     rclpy.init(args=args)
#     yolo_node = YoloNode()

#     try:
#         rclpy.spin(yolo_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         yolo_node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from ultralytics import YOLO    
import numpy as np
import cv2
import time 

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # YOLO模型
        self.model = YOLO("/home/ultralytics/vision-ws/src/ultralytics-ros/weight/ver4.pt")

        # 訂閱相機影像
        self.color_sub = self.create_subscription(Image, '/realsense2/cam_mid/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/realsense2/cam_mid/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        
        # 發布檢測到的物件座標（PoseArray 格式）
        self.center_pub = self.create_publisher(PoseArray, '/detected/cam_pose_array', 10)
        self.center_pub_platform = self.create_publisher(PoseArray, '/detected/cam_pose_array/platform', 10)
        self.center_pub_column = self.create_publisher(PoseArray, '/detected/cam_pose_array/column', 10)


        # CvBridge
        self.bridge = CvBridge()
        
        self.depth_image = None 
        self.color_msg = None

        self.get_logger().info("YOLO Node initialized and ready.")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def color_callback(self, msg):
        self.color_msg = msg
        self.detect_objects()
        
    def detect_objects(self):
        if self.color_msg is None:
            self.get_logger().warning("No image message received yet.")
            return

        # 轉換影像為 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(self.color_msg, desired_encoding='bgr8')

        # 使用 YOLO 進行物件偵測
        results = self.model(cv_image)

        # 初始化 PoseArray
        pose_array = PoseArray()
        pose_array.header.frame_id = 'cam_mid_color_optical_frame'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for object in results:
            boxes = object.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 邊界框座標
                confidence = box.conf[0].item()  
                label = box.cls[0].item()  

                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2

                # 轉換為相機座標系
                pose = self.switch_to_cam_pose(center_x, center_y)
                pose_array.poses.append(pose)

        # 發布 PoseArray
        self.center_pub.publish(pose_array)
        if label == 0:
            self.center_pub_platform.publish(pose_array)
        else:
            self.center_pub_column.publish(pose_array)
        pose_array.poses.clear()

    def switch_to_cam_pose(self, x, y):
        f_x = 609.68  # 內參
        f_y = 608.42
        c_x = 425.38
        c_y = 238.43

        z = self.depth_image[int(y), int(x)] if self.depth_image is not None else 2000

        pose = Pose()
        pose.position.x = (z * (x - c_x) / f_x) / 1000 
        pose.position.y = (z * (y - c_y) / f_y) / 1000
        pose.position.z = z / 1000
        return pose

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

