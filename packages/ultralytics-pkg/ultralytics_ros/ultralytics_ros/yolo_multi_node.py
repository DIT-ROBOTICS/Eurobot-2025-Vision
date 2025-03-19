import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import message_filters  # 新增 message_filters 进行时间同步

class Yolo_Multi_Node(Node):
    def __init__(self):
        super().__init__('yolo_multi_node')

        # 初始化 YOLO
        self.model = YOLO("/home/ultralytics/vision-ws/src/ultralytics-ros/weight/ver5.pt")

        # CvBridge
        self.bridge = CvBridge()

        # 创建同步订阅者
        self.color_sub_mid = message_filters.Subscriber(self, Image, '/realsense2/cam_mid/color/image_raw')
        self.color_sub_left = message_filters.Subscriber(self, Image, '/realsense1/cam_left/color/image_raw')
        self.color_sub_right = message_filters.Subscriber(self, Image, '/realsense3/cam_right/color/image_raw')

        self.depth_sub_mid = message_filters.Subscriber(self, Image, '/realsense2/cam_mid/aligned_depth_to_color/image_raw')
        self.depth_sub_left = message_filters.Subscriber(self, Image, '/realsense1/cam_left/aligned_depth_to_color/image_raw')
        self.depth_sub_right = message_filters.Subscriber(self, Image, '/realsense3/cam_right/aligned_depth_to_color/image_raw')

        # 近似时间同步，queue_size 可调大（如10），slop 设为 0.1 (100ms) 以增加匹配容忍度
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub_mid, self.color_sub_left, self.color_sub_right,
             self.depth_sub_mid, self.depth_sub_left, self.depth_sub_right], 
            queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        # 目标框图像发布者
        self.bbox_pub = self.create_publisher(Image, '/detected/bounding_boxes', 10)

        # 物体位姿发布者
        self.pose_pub = self.create_publisher(PoseArray, '/detected/cam_pose_array', 10)

        self.get_logger().info("YOLO Node with synchronized cameras initialized.")

    def sync_callback(self, img_mid, img_left, img_right, depth_mid, depth_left, depth_right):
        """
        这个函数会在三台相机的图像同步后触发，确保每次 YOLO 处理的是时间匹配的数据
        """
        # 将 ROS Image 转换为 OpenCV 图像
        cv_img_mid = self.bridge.imgmsg_to_cv2(img_mid, desired_encoding='bgr8')
        cv_img_left = self.bridge.imgmsg_to_cv2(img_left, desired_encoding='bgr8')
        cv_img_right = self.bridge.imgmsg_to_cv2(img_right, desired_encoding='bgr8')

        depth_img_mid = self.bridge.imgmsg_to_cv2(depth_mid, desired_encoding='16UC1')
        depth_img_left = self.bridge.imgmsg_to_cv2(depth_left, desired_encoding='16UC1')
        depth_img_right = self.bridge.imgmsg_to_cv2(depth_right, desired_encoding='16UC1')

        # 处理三台相机图像
        self.process_image(cv_img_mid, depth_img_mid, 'cam_mid_color_optical_frame')
        self.process_image(cv_img_left, depth_img_left, 'cam_left_color_optical_frame')
        self.process_image(cv_img_right, depth_img_right, 'cam_right_color_optical_frame')

    def process_image(self, cv_image, depth_image, frame_id):
        """
        处理单张相机图像，执行 YOLO 检测并计算 3D 位姿
        """
        results = self.model(cv_image)
        results_img = results[0].plot()

        # 发送检测框图像
        self.bbox_pub.publish(self.bridge.cv2_to_imgmsg(results_img, encoding="bgr8"))

        pose_array = PoseArray()
        pose_array.header.frame_id = frame_id
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for object in results:
            boxes = object.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                confidence = box.conf[0].item()
                label = box.cls[0].item()

                # 计算 3D 坐标
                posem = self.switch_to_cam_pose((x1 + x2) / 2, (y1 + y2) / 2, depth_image)

                if confidence >= 0.70:
                    pose_array.poses.append(posem)

        # 发布位姿信息
        self.pose_pub.publish(pose_array)

    def switch_to_cam_pose(self, x, y, depth_image):
        """
        将 2D 像素坐标转换为 3D 物理坐标
        """
        f_x, f_y = 457.26, 456.26
        c_x, c_y = 326.35, 177.61
        z = depth_image[int(y), int(x)] if depth_image is not None else 0

        pose = Pose()
        pose.position.x = (z * (x - c_x) / f_x) / 1000
        pose.position.y = (z * (y - c_y) / f_y) / 1000
        pose.position.z = z / 1000

        return pose


def main(args=None):
    rclpy.init(args=args)
    yolo_multi_node = Yolo_Multi_Node()

    try:
        rclpy.spin(yolo_multi_node)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_multi_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
