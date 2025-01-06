import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

def Depth_Contour(depth_cv_image):
    ret, binary = cv2.adaptiveThreshold(depth_cv_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # search all contour
        for contour in contours:
            # calculate all contours if too small then ignore
            area = cv2.contourArea(contour)
            if area > 500:
                # calculate rectangular
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(depth_cv_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                # cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                cv2.imshow("Depth", depth_cv_image)
                cv2.waitKey(1)
    
    return contours

def Color_Contour(color_cv_image):
    gray_image = cv2.cvtColor(color_cv_image, cv2.COLOR_BGR2GRAY)
    ret, binary = cv2.adaptiveThreshold(gray_image, 255, cv.ADAPTIVE_THRESH_GAUSSIAN_C)
    binary = cv2.erode(binary, None, iterations=2)
    binary = cv2.dilate(binary, None, iterations=2)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(binary, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.rectangle(color_cv_image, (x, y), (x + w, y + h), (255, 255, 255), 2)
                # cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                cv2.imshow("Color", color_cv_image)
                cv2.waitKey(1)
                # cv2.imshow("Gray", binary)
                # cv2.waitKey(1)

    return contours

# def Contour(color_cv_image, depth_cv_image):

#     cv2.imshow()

class RealSenseListener(Node):
    def __init__(self):
        super().__init__('realsense_listener')
        
        self.color_subscription = self.create_subscription(
            Image,
            '/cam1/D435i/color/image_raw',
            self.color_callback,
            10
        )

        self.depth_subscription = self.create_subscription(
            Image,
            "/cam1/D435i/depth/image_rect_raw",
            self.depth_callback,
            10
        )

        self.color_image = None
        self.color_contour = None
        self.depth_image = None
        self.depth_contour = None
        
        self.bridge = CvBridge()

    def color_callback(self, msg):
        try:
            color_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            color_normalized_image = cv2.normalize(color_cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
            self.color_image = color_normalized_image
            # print(f"dtype{color_cv_image.dtype}")
            # cv2.imshow("Color", self.color_image)
            # cv2.waitKey(1)
            self.color_contour = Color_Contour(self.color_image)

            # self.get_logger().info(f"Received color image with shape: {self.color_image.shape}")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

    def depth_callback(self, msg):
        try:
            depth_cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth_normalized_image = cv2.normalize(depth_cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC3)
            self.depth_image = depth_normalized_image
            self.depth_contour = Depth_Contour(self.depth_image)

            # self.get_logger().info(f"Received depth image with shape: {self.depth_image.shape}")            
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")


def main(args=None):
    rclpy.init(args=args)

    realsense_listener = RealSenseListener()

    rclpy.spin(realsense_listener)

    realsense_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
