import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
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
VERBOSE = True

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_all_node')
        self.declare_parameter("model_path", "/home/ultralytics/vision-ws/src/ultralytics-ros/weight/ver7_0507.pt")
        self.declare_parameter("color_topic", "/vision/stitched_image/color/image_raw")
        self.declare_parameter("depth_topic", "/vision/stitched_image/depth/image_raw")
        self.declare_parameter("bbox_topic", "/vision/bounding_boxes")
        self.declare_parameter("platform_pose_topic", "/vision/global_center_poses/platform")
        self.declare_parameter("column_pose_topic", "/vision/global_center_poses/column")
        self.declare_parameter("overturn_pose_topic", "/vision/global_center_poses/overturn")
        self.declare_parameter("set_pose_topic", "/vision/global_center_poses/set")
        self.declare_parameter("from_frame_id", "cam_mid_color_optical_frame")
        self.declare_parameter("to_frame_id", "map")
        self.declare_parameter("gui", False)  
        self.declare_parameter("column_confidance", 0.50)
        self.declare_parameter("platform_confidance", 0.80)
        self.declare_parameter("overturn_confidance", 0.50)
        self.declare_parameter("set_confidance", 0.40)
        self.from_frame_id = self.get_parameter("from_frame_id").value
        self.to_frame_id = self.get_parameter("to_frame_id").value
        model_path = self.get_parameter("model_path").value
        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        bbox_topic = self.get_parameter("bbox_topic").value

        platform_pose_topic = self.get_parameter("platform_pose_topic").value
        column_pose_topic = self.get_parameter("column_pose_topic").value
        overturn_pose_topic = self.get_parameter("overturn_pose_topic").value
        set_pose_topic = self.get_parameter("set_pose_topic").value

        self.platform_confidance = self.get_parameter("platform_confidance").value
        self.column_confidance = self.get_parameter("column_confidance").value
        self.overturn_confidance = self.get_parameter("overturn_confidance").value
        self.set_confidance = self.get_parameter("set_confidance").value

        self.gui = self.get_parameter("gui").value 
        self.listener_qos = self._create_qos_profile()
        self.model = YOLO(model_path)
        self.get_logger().info(f"Loaded YOLO model from {model_path}")
        self.get_logger().info(f"Color topic: {color_topic}")
        self.get_logger().info(f"Depth topic: {depth_topic}")
        self.depth_msg = None 
        self.color_msg = None
        self.color_sub = self.create_subscription(Image, color_topic, self.color_callback, self.listener_qos)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, self.listener_qos)
        self.center_pub_platform = self.create_publisher(PoseArray, platform_pose_topic, 10)
        self.center_pub_column = self.create_publisher(PoseArray, column_pose_topic, 10)
        self.center_pub_overturn = self.create_publisher(PoseArray, overturn_pose_topic, 10)
        self.center_pub_set = self.create_publisher(PoseArray, set_pose_topic, 10)
        self.bbox_pub = self.create_publisher(Image, bbox_topic, 10)
        self.bridge = CvBridge()
        self.get_logger().info("YOLO Node initialized and ready.")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transformer = PoseTransformer(self.tf_buffer, self.from_frame_id, self.to_frame_id)
        self.counter_recognition = CounterRecognition()
        self.last_color_msg_time = self.get_clock().now()
        self.last_depth_msg_time = self.get_clock().now()
        self.watchdog_timer = self.create_timer(1.0, self.check_image_msg_timeout)
        # self.fps = 0.0
        # self.timer = self.create_timer(1.0, self.print_fps)


    def depth_callback(self, msg):
        self.depth_msg = msg
        self.last_depth_msg_time = self.get_clock().now()
        self.run_if_ready()

    def color_callback(self, msg):
        self.color_msg = msg
        self.last_color_msg_time = self.get_clock().now()
        self.run_if_ready()
    
    def check_image(self, color_image, depth_image):
        if color_image is None or depth_image is None:
            self.get_logger().error("Received empty image")
            return False
        return True

    def run_if_ready(self):
        if self.color_msg is not None and self.depth_msg is not None:
            self.predict()
            self.color_msg = None  
            self.depth_msg = None
    def check_image_msg_timeout(self):
        now = self.get_clock().now()
        color_time_diff = now - self.last_color_msg_time
        depth_time_diff = now - self.last_depth_msg_time
        if color_time_diff.nanoseconds * 1e-9 > 3.0:
            self.get_logger().error("No color image received!!!")
        if depth_time_diff.nanoseconds * 1e-9 > 3.0:    
            self.get_logger().error("No depth image received!!!")

    def predict(self):
        # start_time = time.time()
        if self.color_msg is not None and self.depth_msg is not None:
            color_image, depth_image = self.preprocess(self.color_msg, self.depth_msg)
            results = self.model(color_image, verbose=VERBOSE, device="cuda")
            results_img = results[0].plot()
            if self.gui:
                self.bbox_pub.publish(results_img)
            pose_array_platform = PoseArray()
            pose_array_column = PoseArray()
            pose_array_overturn = PoseArray()
            pose_array_set = PoseArray()
            pose_array_platform.header.frame_id = self.from_frame_id
            pose_array_platform.header.stamp = self.get_clock().now().to_msg()
            pose_array_column.header.frame_id = self.to_frame_id
            pose_array_column.header.stamp = self.get_clock().now().to_msg()
            pose_array_overturn.header.frame_id = self.to_frame_id
            pose_array_overturn.header.stamp = self.get_clock().now().to_msg()
            pose_array_set.header.frame_id = self.to_frame_id
            pose_array_set.header.stamp = self.get_clock().now().to_msg()
            for object in results:
                boxes = object.boxes
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  
                    x, y = (x1 + x2) / 2, (y1 + y2) / 2
                    z = depth_image[int(y-1), int(x-1)] if depth_image is not None else 0
                    confidence = box.conf[0].item()  
                    label_id = int(box.cls[0].item())
                    label_name = self.model.names[label_id]
                    if label_name == "platform" and confidence >= self.platform_confidance:
                        posem = self.transformer.switch_to_cam_pose(x, y, z) 
                        try:
                            global_posem = self.transformer.transform_pose(posem)
                            cropped_img = color_image[y1:y2, x1:x2]
                            self.counter_recognition.set_image(cropped_img)
                            binary_img = self.counter_recognition.get_binary_img_plat()
                            self.counter_recognition.get_contours()
                            theangle = self.counter_recognition.distinguish_contour(cropped_img.copy(), global_posem, "platform") 
                            finalpose = global_posem
                            finalpose.orientation.x = theangle
                            finalpose.orientation.y = 0.0
                            finalpose.orientation.z = math.sin(theangle / 2)
                            finalpose.orientation.w = math.cos(theangle / 2)
                            pose_array_platform.poses.append(finalpose)                      
                        except Exception as e:
                            self.get_logger().error(f"Transform failed: {str(e)}")
                    elif label_name == "overturn" and confidence >= self.overturn_confidance:
                        posem = self.transformer.switch_to_cam_pose(x, y, z)                       
                        try:
                            global_posem = self.transformer.transform_pose(posem)
                            cropped_img = color_image[y1:y2, x1:x2]
                            self.counter_recognition.set_image(cropped_img)
                            binary_img = self.counter_recognition.get_binary_img_colu()
                            self.counter_recognition.get_contours()
                            theangle = self.counter_recognition.distinguish_contour(cropped_img.copy(), global_posem, "overturn")  
                            finalpose = global_posem
                            finalpose.orientation.x = 0.0
                            finalpose.orientation.y = 0.0
                            finalpose.orientation.z = math.sin(theangle / 2)
                            finalpose.orientation.w = math.cos(theangle / 2)
                            pose_array_overturn.poses.append(finalpose)                     
                        except Exception as e:
                            self.get_logger().error(f"Transform failed: {str(e)}")                          
                    elif label_name == "column" and confidence >= self.column_confidance:
                        posem = self.transformer.switch_to_cam_pose(x, y, z)
                        try:
                            global_posem = self.transformer.transform_pose(posem) 
                            finalpose = global_posem
                            finalpose.orientation.x = 0.0
                            finalpose.orientation.y = 0.0
                            finalpose.orientation.z = 0.0
                            finalpose.orientation.w = 1.0
                            pose_array_column.poses.append(finalpose)  
                        except Exception as e:
                            self.get_logger().error(f"Transform failed: {str(e)}")
                    elif label_name == "set" and confidence >= self.set_confidance:
                        posem = self.transformer.switch_to_cam_pose(x, y, z)
                        try:
                            global_posem = self.transformer.transform_pose(posem) 
                            finalpose = global_posem
                            finalpose.orientation.x = 0.0
                            finalpose.orientation.y = 0.0
                            finalpose.orientation.z = 0.0
                            finalpose.orientation.w = 1.0
                            pose_array_set.poses.append(finalpose)  
                        except Exception as e:
                            self.get_logger().error(f"Transform failed: {str(e)}")
            self.center_pub_platform.publish(pose_array_platform)
            self.center_pub_column.publish(pose_array_column)
            self.center_pub_overturn.publish(pose_array_overturn)
            self.center_pub_set.publish(pose_array_set)
            # end_time = time.time()
            # self.fps = 1.0 / (end_time - start_time)
            pose_array_column.poses.clear()
            pose_array_platform.poses.clear()
            pose_array_overturn.poses.clear()

    def preprocess(self, color_image, depth_image):
        try:
            col = self.bridge.imgmsg_to_cv2(color_image, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Failed to process color image: {e}")
        try:
            dep = self.bridge.imgmsg_to_cv2(depth_image, desired_encoding='16UC1')
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")
        return col, dep

    def _create_qos_profile(self):
        return QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )

    def print_fps(self):
        self.get_logger().info(f'FPS: {self.fps:.2f}')

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YoloNode()
    executor = MultiThreadedExecutor()
    executor.add_node(yolo_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        yolo_node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()