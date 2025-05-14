import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from ultralytics import YOLO
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import numpy as np
import cv2
import math
import time
import threading
import queue
from .importClass.angle import CounterRecognition
from .importClass.tf_transform import PoseTransformer

VERBOSE = False

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node_nopub')
        # 宣告參數
        self.declare_parameter("model_path", "/home/ultralytics/vision-ws/src/ultralytics-ros/weight/ver7_0507.pt")
        self.declare_parameter("color_topic", "/vision/stitched_image/color/image_raw")
        self.declare_parameter("depth_topic", "/vision/stitched_image/depth/image_raw")
        self.declare_parameter("bbox_topic", "/vision/bounding_boxes")
        self.declare_parameter("platform_pose_topic", "/vision/global_center_poses/platform")
        self.declare_parameter("column_pose_topic", "/vision/global_center_poses/column")
        self.declare_parameter("overturn_pose_topic", "/vision/global_center_poses/overturn")
        self.declare_parameter("from_frame_id", "cam_mid_color_optical_frame")
        self.declare_parameter("to_frame_id", "map")

        # 獲取參數
        self.from_frame_id = self.get_parameter("from_frame_id").value
        self.to_frame_id = self.get_parameter("to_frame_id").value
        model_path = self.get_parameter("model_path").value
        color_topic = self.get_parameter("color_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        bbox_topic = self.get_parameter("bbox_topic").value
        platform_pose_topic = self.get_parameter("platform_pose_topic").value
        column_pose_topic = self.get_parameter("column_pose_topic").value
        overturn_pose_topic = self.get_parameter("overturn_pose_topic").value

        # QoS 配置
        self.listener_qos = self._create_qos_profile()

        # 初始化 YOLO 模型
        self.model = YOLO(model_path).to("cuda")
        self.get_logger().info(f"已載入 YOLO 模型: {model_path}")

        # 訂閱者
        self.color_msg = None
        self.depth_msg = None
        self.color_sub = self.create_subscription(Image, color_topic, self.color_callback, self.listener_qos)
        self.depth_sub = self.create_subscription(Image, depth_topic, self.depth_callback, self.listener_qos)

        # 發布者
        self.center_pub_platform = self.create_publisher(PoseArray, platform_pose_topic, 10)
        self.center_pub_column = self.create_publisher(PoseArray, column_pose_topic, 10)
        self.center_pub_overturn = self.create_publisher(PoseArray, overturn_pose_topic, 10)
        self.bbox_pub = self.create_publisher(Image, bbox_topic, 10)

        # 初始化 CvBridge、TF 等組件
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transformer = PoseTransformer(self.tf_buffer, self.from_frame_id, self.to_frame_id)
        self.counter_recognition = CounterRecognition()

        # 執行緒安全的佇列，用於傳遞影像給檢測執行緒
        self.image_queue = queue.Queue(maxsize=1)  # 限制佇列大小以避免記憶體問題
        self.shutdown_event = threading.Event()

        # 啟動檢測執行緒
        self.detection_thread = threading.Thread(target=self.detection_loop, daemon=True)
        self.detection_thread.start()

        self.timer = self.create_timer(2.0, self.check_image_status)
        self.get_logger().info("YOLO 節點已初始化並準備就緒。")

    def _create_qos_profile(self):
        return QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE
        )

    def depth_callback(self, msg):
        self.depth_msg = msg
        self.run_if_ready()

    def color_callback(self, msg):
        self.color_msg = msg
        self.run_if_ready()

    def run_if_ready(self):
        if self.color_msg is not None and self.depth_msg is not None:
            # 在主執行緒中預處理影像，避免 CvBridge 的執行緒安全問題
            try:
                color_image = self.bridge.imgmsg_to_cv2(self.color_msg, desired_encoding='bgr8')
                depth_image = self.bridge.imgmsg_to_cv2(self.depth_msg, desired_encoding='16UC1')
                # 將影像放入佇列，供檢測執行緒處理
                try:
                    self.image_queue.put_nowait((color_image, depth_image))
                except queue.Full:
                    self.get_logger().warn("影像佇列已滿，跳過此幀")
            except Exception as e:
                self.get_logger().error(f"預處理影像失敗: {e}")
            finally:
                # 清空訊息以接收新訊息
                self.color_msg = None
                self.depth_msg = None

    def detection_loop(self):
        """在獨立執行緒中運行 YOLO 檢測"""
        while not self.shutdown_event.is_set():
            try:
                # 從佇列中獲取影像（阻塞直到有資料）
                color_image, depth_image = self.image_queue.get(timeout=1.0)
                self.predict(color_image, depth_image)
                self.image_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"檢測執行緒錯誤: {e}")

    def predict(self, color_image, depth_image):
        start_time = time.time()
        # 執行 YOLO 推理
        results = self.model(color_image, verbose=VERBOSE, device="cuda")
        results_img = results[0].plot()
        self.bbox_pub.publish(self.bridge.cv2_to_imgmsg(results_img, encoding="bgr8"))

        # 初始化 PoseArray
        pose_array_platform = PoseArray()
        pose_array_column = PoseArray()
        pose_array_overturn = PoseArray()
        pose_array_platform.header.frame_id = self.from_frame_id
        pose_array_platform.header.stamp = self.get_clock().now().to_msg()
        pose_array_column.header.frame_id = self.to_frame_id
        pose_array_column.header.stamp = self.get_clock().now().to_msg()
        pose_array_overturn.header.frame_id = self.to_frame_id
        pose_array_overturn.header.stamp = self.get_clock().now().to_msg()

        # 處理檢測結果
        for object in results:
            boxes = object.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                (x, y) = ((x1 + x2) / 2, (y1 + y2) / 2)
                z = depth_image[int(y-1), int(x-1)] if depth_image is not None else 0
                confidence = box.conf[0].item()
                label_id = int(box.cls[0].item())
                label_name = self.model.names[label_id]

                if label_name == "platform" and confidence >= 0.60:
                    posem = self.transformer.switch_to_cam_pose(x, y, z)
                    try:
                        global_posem = self.transformer.transform_pose(posem)
                        cropped_img = color_image[y1:y2, x1:x2]
                        self.counter_recognition.set_image(cropped_img)
                        binary_img = self.counter_recognition.get_binary_img_plat()
                        self.counter_recognition.get_contours()
                        theangle = self.counter_recognition.distinguish_contour(cropped_img.copy(), global_posem, "platform")
                    except Exception as e:
                        self.get_logger().error(f"變換失敗: {e}")
                        continue
                    if global_posem is not None:
                        finalpose = Pose()
                        finalpose = global_posem
                        finalpose.orientation.x = theangle
                        finalpose.orientation.y = 0.0
                        finalpose.orientation.z = math.sin(theangle / 2)
                        finalpose.orientation.w = math.cos(theangle / 2)
                        pose_array_platform.poses.append(finalpose)

                elif label_name == "overturn" and confidence >= 0.40:
                    posem = self.transformer.switch_to_cam_pose(x, y, z)
                    try:
                        global_posem = self.transformer.transform_pose(posem)
                        cropped_img = color_image[y1:y2, x1:x2]
                        self.counter_recognition.set_image(cropped_img)
                        binary_img = self.counter_recognition.get_binary_img_colu()
                        self.counter_recognition.get_contours()
                        theangle = self.counter_recognition.distinguish_contour(cropped_img.copy(), global_posem, "overturn")
                    except Exception as e:
                        self.get_logger().error(f"變換失敗: {e}")
                        continue
                    if global_posem is not None:
                        finalpose = Pose()
                        finalpose = global_posem
                        finalpose.orientation.x = theangle
                        finalpose.orientation.y = 0.0
                        finalpose.orientation.z = math.sin(theangle / 2)
                        finalpose.orientation.w = math.cos(theangle / 2)
                        pose_array_overturn.poses.append(finalpose)

                elif label_name == "column" and confidence >= 0.40:
                    posem = self.transformer.switch_to_cam_pose(x, y, z)
                    try:
                        global_posem = self.transformer.transform_pose(posem)
                    except Exception as e:
                        self.get_logger().error(f"變換失敗: {e}")
                        continue
                    if global_posem is not None:
                        finalpose = Pose()
                        finalpose = global_posem
                        finalpose.orientation.x = 0.0
                        finalpose.orientation.y = 0.0
                        finalpose.orientation.z = 0.0
                        finalpose.orientation.w = 1.0
                        pose_array_column.poses.append(finalpose)

        # 發布結果
        self.center_pub_platform.publish(pose_array_platform)
        self.center_pub_column.publish(pose_array_column)
        self.center_pub_overturn.publish(pose_array_overturn)

        # 記錄 FPS
        end_time = time.time()
        fps = 1.0 / (end_time - start_time)
        self.get_logger().info(f"推理 FPS: {fps:.2f}")

    def check_image_status(self):
        if self.color_msg is None:
            self.get_logger().warn("仍在等待彩色影像...")
        if self.depth_msg is None:
            self.get_logger().warn("仍在等待深度影像...")

    def destroy_node(self):
        self.shutdown_event.set()
        self.detection_thread.join(timeout=2.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    yolo_node_nopub = YoloNode()
    # 使用 MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)  # 可根據需求調整執行緒數
    executor.add_node(yolo_node_nopub)
    try:
        rclpy.spin(yolo_node_nopub)
    except KeyboardInterrupt:
        yolo_node_nopub.get_logger().info('KeyboardInterrupt received, shutting down...')
    except Exception as e:
        yolo_node_nopub.get_logger().error(f'Error during spin: {e}')
    finally:
        yolo_node_nopub.destroy_node()
        if rclpy.ok():  # Check if context is still valid
            try:
                rclpy.shutdown()
                yolo_node_nopub.get_logger().info('ROS context shut down successfully')
            except Exception as e:
                yolo_node_nopub.get_logger().error(f'Error during shutdown: {e}')

if __name__ == '__main__':
    main()