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
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
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
        pose_array_platform = PoseArray()
        pose_array_column = PoseArray()
        pose_array_platform.header.frame_id = 'cam_mid_color_optical_frame'
        pose_array_platform.header.stamp = self.get_clock().now().to_msg()
        pose_array_column.header.frame_id = 'cam_mid_color_optical_frame'
        pose_array_column.header.stamp = self.get_clock().now().to_msg()
        for object in results:
            boxes = object.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 邊界框座標
                confidence = box.conf[0].item()  
                label = box.cls[0].item()  

                pose1 = self.switch_to_cam_pose(x1, y1)
                pose2 = self.switch_to_cam_pose(x2, y2)
                print(f"Detected object: x1={x1}, y1={y1}, x2={x2}, y2={y2}, \n")
                print(f"confidence={confidence:.2f}, label={label}\n")
                if(confidence >=0.70):
                    if(label==0):
                        pose_array_platform.poses.append(pose1)
                        pose_array_platform.poses.append(pose2)
                    elif(label==1):
                        pose_array_column.poses.append(pose1)
                        pose_array_column.poses.append(pose1)
                    # if pose_array_platform and hasattr(pose_array_platform, 'poses'):
                    #     print(f"PoseArray_platform has {len(pose_array_platform.poses)} objects")
                    # else:
                    #     print("PoseArray_platform is None or invalid")


        self.center_pub_platform.publish(pose_array_platform)
        self.center_pub_column.publish(pose_array_column)
        pose_array_column.poses.clear()
        pose_array_platform.poses.clear()



    def switch_to_cam_pose(self, x, y):
        f_x = 457.26  # 內參
        f_y = 456.26
        c_x = 326.35
        c_y = 177.61

        z = self.depth_image[int(y), int(x)] if self.depth_image is not None else 0

        pose = Pose()
        pose.position.x = (z * (x - c_x) / f_x) / 1000 
        pose.position.y = (z * (y - c_y) / f_y) / 1000
        pose.position.z = z / 1000
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
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

