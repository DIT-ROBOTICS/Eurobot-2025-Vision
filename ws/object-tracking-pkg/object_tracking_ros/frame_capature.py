import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class FirstFrameCapture(Node):
    def __init__(self):
        super().__init__('first_frame_capture')
        
        # creat folder
        self.image_dir = 'src/object-tracking-ros/image'
        os.makedirs(self.image_dir, exist_ok=True)
        
        self.bridge = CvBridge()
        self.color_frame = None
        self.depth_frame = None
        self.got_color_frame = False
        self.got_depth_frame = False
        
        # subscriber
        self.color_sub = self.create_subscription(
            Image,
            '/realsense1/cam/color/image_raw',
            self.color_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/realsense1/cam/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )

    def color_callback(self, msg):
        if not self.got_color_frame:
            try:
                color_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.color_frame = color_frame
                
                # path
                color_path = os.path.join(self.image_dir, 'first_color_frame.png')
                cv2.imwrite(color_path, color_frame)
                self.get_logger().info(f'Has saved color frame: {color_path}')
                
                self.got_color_frame = True
                
                # check
                if self.got_color_frame and self.got_depth_frame:
                    rclpy.shutdown()
                
            except Exception as e:
                self.get_logger().error(f'Error when processing color frame: {str(e)}')

    def depth_callback(self, msg):
        if not self.got_depth_frame:
            try:
                depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                depth_frame_normalized = cv2.normalize(depth_frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                self.depth_frame = depth_frame_normalized
                
                # path
                depth_path = os.path.join(self.image_dir, 'first_depth_frame.png')
                cv2.imwrite(depth_path, depth_frame_normalized)
                self.get_logger().info(f'Has saved depth frame: {depth_path}')
                
                self.got_depth_frame = True
                
                # check
                if self.got_color_frame and self.got_depth_frame:
                    rclpy.shutdown()
                
            except Exception as e:
                self.get_logger().error(f'Error when processing depth frame: {str(e)}')

# def main():
#     rclpy.init()
#     node = FirstFrameCapture()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()