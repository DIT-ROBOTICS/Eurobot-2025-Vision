import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO    
import numpy as np
import cv2
import time 
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.from_frame_id = self.get_parameter("from_frame_id").value
        self.to_frame_id = self.get_parameter("to_frame_id").value
        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO model from {model_path}")
        self.color_sub = self.create_subscription(Image, color_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.center_pub_platform = self.create_publisher(PoseArray, platform_pose_topic, 10)
        self.center_pub_column = self.create_publisher(PoseArray, column_pose_topic, 10)
        self.bbox_pub = self.create_publisher(Image, bbox_topic, 10)
        self.bridge = CvBridge()
        self.depth_image = None 
        self.color_msg = None
        self.get_logger().info("YOLO Node initialized and ready.")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def color_callback(self, msg):
        self.color_msg = msg
        if self.color_msg is None:
            self.get_logger().warning("No image message received yet.")
            return

        # 轉換影像為 OpenCV 格式
        cv_image = self.bridge.imgmsg_to_cv2(self.color_msg, desired_encoding='bgr8')

        # 使用 YOLO 進行物件偵測
        results = self.model(cv_image)
        results_img = results[0].plot()
        self.bbox_pub.publish(self.bridge.cv2_to_imgmsg(results_img, encoding="bgr8"))
        # 初始化 PoseArray
        pose_array_platform = PoseArray()
        pose_array_column = PoseArray()
        global_poses_platform = PoseArray()
        global_poses_column = PoseArray()
        pose_array_platform.header.frame_id = self.from_frame_id
        pose_array_platform.header.stamp = self.get_clock().now().to_msg()
        pose_array_column.header.frame_id = self.from_frame_id
        pose_array_column.header.stamp = self.get_clock().now().to_msg()
        for object in results:
            boxes = object.boxes
            for box in boxes:
                
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 邊界框座標
                confidence = box.conf[0].item()  
                label = box.cls[0].item()  

                if(confidence >=0.70):
                    pose1 = self.switch_to_cam_pose(x1, y1)
                    pose2 = self.switch_to_cam_pose(x2, y2)
                    posem = self.switch_to_cam_pose((x1+x2)/2,(y1+y2)/2)
                    if(label==0):
                        try:
                            global_pose1 = self.transform_pose(self.from_frame_id,self.to_frame_id,pose1)
                            global_pose2 = self.transform_pose(self.from_frame_id,self.to_frame_id,pose2)
                            global_posem = self.transform_pose(self.from_frame_id,self.to_frame_id,posem)
                        except Exception as e:
                            self.get_logger().error(f"Transform failed: {str(e)}")
                        # global_pose1 = self.transform_pose(self.from_frame_id,self.to_frame_id,pose1)
                        # global_pose2 = self.transform_pose(self.from_frame_id,self.to_frame_id,pose2)
                        # global_posem = self.transform_pose(self.from_frame_id,self.to_frame_id,posem)
                        if(global_pose1 is not None):
                            finalpose = Pose()
                            finalpose.position.x = (global_pose1.position.x + global_pose2.position.x)/2
                            finalpose.position.y = (global_pose1.position.y + global_pose2.position.y)/2
                            finalpose.position.z = global_posem.position.z
                            length = (global_pose1.position.x - global_pose2.position.x)*100
                            height = (global_pose1.position.y - global_pose2.position.y)*100
                            if(length<=10):
                                length=10.00001
                            elif(length>41.231056):
                                length=41.231056
                            if(height<=10):
                                height=10.00001
                            elif(height>41.231056):
                                height=41.231056

                            if (length>=height):
                                angle = math.acos(height/41.231056)+1.3258176
                            else:
                                angle = math.acos(length/41.231056)+0.2449786
                            print(f"angle = {angle}")
                            finalpose.orientation.x = 0.0
                            finalpose.orientation.y = 0.0
                            finalpose.orientation.z = math.sin(angle / 2)
                            finalpose.orientation.w = math.cos(angle / 2)
                            pose_array_platform.poses.append(finalpose)  

                    elif(label==1):
                        try:
                            global_pose1 = self.transform_pose(self.from_frame_id,self.to_frame_id,pose1)
                            global_pose2 = self.transform_pose(self.from_frame_id,self.to_frame_id,pose2)
                            global_posem = self.transform_pose(self.from_frame_id,self.to_frame_id,posem)
                        except Exception as e:
                            self.get_logger().error(f"Transform failed: {str(e)}")
                        # global_pose1 = self.transform_pose(self.from_frame_id,self.to_frame_id,pose1)
                        # global_pose2 = self.transform_pose(self.from_frame_id,self.to_frame_id,pose2)
                        # global_posem = self.transform_pose(self.from_frame_id,self.to_frame_id,posem)
                        if(global_pose1 is not None):
                            finalpose = Pose()
                            finalpose.position.x = (global_pose1.position.x + global_pose2.position.x)/2
                            finalpose.position.y = (global_pose1.position.y + global_pose2.position.y)/2
                            finalpose.position.z = global_posem.position.z
                            finalpose.orientation.x = 0.0
                            finalpose.orientation.y = 0.0
                            finalpose.orientation.z = 0.0
                            finalpose.orientation.w = 1.0
                            pose_array_column.poses.append(finalpose)
        self.center_pub_platform.publish(pose_array_platform)
        self.center_pub_column.publish(pose_array_column)

        pose_array_column.poses.clear()
        pose_array_platform.poses.clear()

    def switch_to_cam_pose(self, x, y): 
        f_x = 457.26  # 內參
        f_y = 456.26
        c_x = 326.35
        c_y = 177.61

        z = self.depth_image[int(y-1), int(x-1)] if self.depth_image is not None else 0


        pose = Pose()
        pose.position.x = (z * (x - c_x) / f_x) / 1000 
        pose.position.y = (z * (y - c_y) / f_y) / 1000
        pose.position.z = z / 1000
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 1.0
        return pose

    def transform_pose(self, from_frame, to_frame, pose):
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = from_frame
            pose_stamped.header.stamp = rclpy.time.Time(seconds=0).to_msg()
            pose_stamped.pose = pose

            # if not self.tf_buffer.can_transform(to_frame, from_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0)):
            #     self.get_logger().error(f"TF transform from {from_frame} to {to_frame} is not available.")
            #     return None

            try:
                transformed_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, timeout=rclpy.duration.Duration(seconds=2.0))
                return transformed_pose_stamped.pose
            except Exception as e:
                self.get_logger().error(f"TF transform failed: {str(e)}")
                return None

            
            return transformed_pose_stamped.pose
        except Exception as e:
            self.get_logger().error(f"Pose transform failed: {str(e)}")
            return None

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

