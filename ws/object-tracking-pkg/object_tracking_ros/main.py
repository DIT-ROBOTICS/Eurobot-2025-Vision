import object_tracking_ros as ob

import rclpy
import numpy as np
import cv2
import os

def main(args=None):
    # Frame Capture
    rclpy.init(args=args)
    frame = ob.FirstFrameCapture()
    while rclpy.ok():
        rclpy.spin_once(frame)
        if frame.got_color_frame and frame.got_depth_frame:
            rclpy.shutdown()
    frame.destroy_node()

    # Contour
    rclpy.init(args=args)
    while rclpy.ok():
        ob.Depth_Contour('src/object-tracking-ros/image/first_depth_frame.png')
        ob.Color_Contour('src/object-tracking-ros/image/first_color_frame.png')

if __name__ == '__main__':
    main()