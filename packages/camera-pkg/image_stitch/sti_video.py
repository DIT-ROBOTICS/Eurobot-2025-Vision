from image_stitch.importClass import MultiCamNode
from image_stitch.importClass import VideoStitcher
from rclpy.executors import MultiThreadedExecutor
import rclpy

def main(args=None):
    rclpy.init(args=args)
    node = MultiCamNode()
    stitcher = VideoStitcher()
    node.get_logger().info('MultiCam Stitching node started')

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            synced_images = node.get_queue_images()
            if synced_images is None:
                continue
            stitched_img = stitcher.warp(synced_images)
            node.publish_stitched_image(stitched_img)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()