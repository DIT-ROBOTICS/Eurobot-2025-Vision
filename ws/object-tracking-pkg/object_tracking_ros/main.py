import object_tracking_ros as ob

import rclpy
import numpy as np
import cv2
import os

def main(args=None):
    rclpy.init(args=args)

    # frame = ob.FirstFrameCapture()

    while rclpy.ok:
        # rclpy.spin_once()
        ob.Depth_Contour('src/object-tracking-ros/image/first_depth_frame.png')

    # rclpy.spin(frame)

    # color_frame = FirstFrameCapture.

    # frame.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()