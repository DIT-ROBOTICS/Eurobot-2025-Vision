from image_stitch.importClass import MultiCamNode
from image_stitch.importClass import VideoStitcher
import rclpy
import threading

def main(args=None):
    rclpy.init(args=args)
    node = MultiCamNode()
    stitcher = VideoStitcher()
    node.get_logger().info('MultiCam Stitching node started')

    try:
        callback_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        callback_thread.start()

        while rclpy.ok():
            synced_images = node.get_queue_images()
            if synced_images is None:
                continue
            stitched_img = stitcher.warp(synced_images)
            if node.encoding == "16UC1":
                stitched_img = stitcher.depth_cali(stitched_img)
            node.publish_stitched_image(stitched_img)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")

    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown() 

if __name__ == '__main__':
    main()