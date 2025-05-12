import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
import cv2
import math
import time
from .importClass.angle import CounterRecognition
from .importClass.tf_transform import PoseTransformer
class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.declare_parameter("model_path", "/home/ultralytics/vision-ws/src/ultralytics-ros/weight/ver7_0507.pt")
        self.declare_parameter("color_topic", "/vision/stitched_image/color/image_raw")
        self.declare_parameter("depth_topic", "/vision/stitched_image/depth/image_raw")
        self.declare_parameter("bbox_topic", "/vision/bounding_boxes")
        self.declare_parameter("platform_pose_topic", "/vision/global_center_poses/platform")
        self.declare_parameter("column_pose_topic", "/vision/global_center_poses/column")
        self.declare_parameter("from_frame_id", "cam_mid_color_optical_frame")
        self.declare_parameter("to_frame_id", "map")
        self.from_frame_id = self.get_parameter("from_frame_id").value
        self.to_frame_id = self.get_parameter("to_frame_id").value
        model_path = self.get_parameter("model_path").value
        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        bbox_topic = self.get_parameter("bbox_topic").value
        platform_pose_topic = self.get_parameter("platform_pose_topic").value
        column_pose_topic = self.get_parameter("column_pose_topic").value
        self.listener_qos = self._create_qos_profile()
        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO model from {model_path}")
        self.depth_msg = None 
        self.color_msg = None
        self.color_sub = self.create_subscription(Image, color_topic, self.color_callback, self.listener_qos)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, self.listener_qos)
        self.center_pub_platform = self.create_publisher(PoseArray, platform_pose_topic, 10)
        self.center_pub_column = self.create_publisher(PoseArray, column_pose_topic, 10)
        self.bbox_pub = self.create_publisher(Image, bbox_topic, 10)
        self.bridge = CvBridge()
        self.get_logger().info("YOLO Node initialized and ready.")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transformer = PoseTransformer(self.tf_buffer, self.from_frame_id, self.to_frame_id)
        self.counter_recognition = CounterRecognition()
        self.timer = self.create_timer(2.0, self.check_image_status)
    def depth_callback(self, msg):
        self.depth_msg = msg
        self.predict()
    def color_callback(self, msg):
        self.color_msg = msg
        self.predict()
    def predict(self):
        if self.color_msg is not None and self.depth_msg is not None:
            color_image, depth_image = self.preprocess(self.color_msg, self.depth_msg)
            results = self.model(color_image)
            results_img = results[0].plot()
            self.bbox_pub.publish(self.bridge.cv2_to_imgmsg(results_img, encoding="bgr8"))
            pose_array_platform = PoseArray()
            pose_array_column = PoseArray()
            global_poses_platform = PoseArray()
            global_poses_column = PoseArray()
            pose_array_platform.header.frame_id = self.from_frame_id
            pose_array_platform.header.stamp = self.get_clock().now().to_msg()
            pose_array_column.header.frame_id = self.to_frame_id
            pose_array_column.header.stamp = self.get_clock().now().to_msg()
            for object in results:
                boxes = object.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  
                    (x,y)=((x1+x2)/2,(y1+y2)/2)
                    z = depth_image[int(y-1), int(x-1)] if depth_image is not None else 0
                    confidence = box.conf[0].item()  
                    label_id = int(box.cls[0].item())
                    label_name = self.model.names[label_id]
                    if(label_name =="platform" and confidence >= 0.60):
                        posem = self.transformer.switch_to_cam_pose(x,y,z)
                        
                        try:
                            global_posem = self.transformer.transform_pose(posem)
                            cropped_img = color_image[y1:y2, x1:x2]
                            self.counter_recognition.set_image(cropped_img)
                            binary_img = self.counter_recognition.get_binary_img_plat()
                            self.counter_recognition.get_contours()
                            theangle = self.counter_recognition.distinguish_contour(cropped_img.copy(),global_posem,"platform")                      
                        except Exception as e:
                            self.get_logger().error(f"Transform failed: {str(e)}")
                        if(global_posem is not None):
                            finalpose = Pose()
                            finalpose = global_posem
                            finalpose.orientation.x = theangle
                            finalpose.orientation.y = 0.0
                            finalpose.orientation.z = math.sin(theangle / 2)
                            finalpose.orientation.w = math.cos(theangle / 2)
                            pose_array_platform.poses.append(finalpose)  
                    
                    elif(label_name =="overturn" and confidence >= 0.40):
                        posem = self.transformer.switch_to_cam_pose(x,y,z)
                        
                        try:
                            global_posem = self.transformer.transform_pose(posem)
                            cropped_img = color_image[y1:y2, x1:x2]
                            self.counter_recognition.set_image(cropped_img)
                            binary_img = self.counter_recognition.get_binary_img_colu()
                            self.counter_recognition.get_contours()
                            theangle = self.counter_recognition.distinguish_contour(cropped_img.copy(),global_posem,"overturn")                      
                        except Exception as e:
                            self.get_logger().error(f"Transform failed: {str(e)}")
                        if(global_posem is not None):
                            finalpose = Pose()
                            finalpose = global_posem
                            finalpose.orientation.x = theangle
                            finalpose.orientation.y = 0.0
                            finalpose.orientation.z = math.sin(theangle / 2)
                            finalpose.orientation.w = math.cos(theangle / 2)
                            pose_array_column.poses.append(finalpose) 

                    elif((label_name=="column")and confidence >= 0.40):
                        posem = self.transformer.switch_to_cam_pose(x,y,z)
                        try:
                            global_posem = self.transformer.transform_pose(posem)
    
                        except Exception as e:
                            self.get_logger().error(f"Transform failed: {str(e)}")
                        if(global_posem is not None):
                            finalpose = Pose()
                            finalpose = global_posem
                            finalpose.orientation.x = 0.0
                            finalpose.orientation.y = 0.0
                            finalpose.orientation.z = 0.0
                            finalpose.orientation.w = 1.0
                            pose_array_column.poses.append(finalpose)

            self.center_pub_platform.publish(pose_array_platform)
            self.center_pub_column.publish(pose_array_column)

            pose_array_column.poses.clear()
            pose_array_platform.poses.clear()
    def preprocess(self,color_image,depth_image):
        try:
            col= self.bridge.imgmsg_to_cv2(color_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")
        try:
            dep = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")
        return col,dep
    def _create_qos_profile(self):
        return QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )
    def check_image_status(self):
        if self.color_msg is None:
            self.get_logger().warn("Still waiting for color image...")
        if self.depth_msg is None:
            self.get_logger().warn("Still waiting for depth image...")
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