from image_stitch.importClass import MultiCamNode
from image_stitch.importClass import VideoStitcher
import rclpy
import time
from queue import Empty
import cv2

def main(args=None):
    rclpy.init(args=args)
    node = MultiCamNode()
    stitcher = VideoStitcher()
    node.get_logger().info('MultiCam Stitching node started')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            images = node.get_images()
            if images is None:
                continue
            stitched_img = stitcher.warp(images)
            node.publish_stitched_image(stitched_img)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()