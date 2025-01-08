from object_tracking_ros import FirstFrameCapture
import rclpy

def main(args=None):
    rclpy.init(args=args)

    realsense_listener = FirstFrameCapture()

    rclpy.spin(realsense_listener)

    realsense_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()