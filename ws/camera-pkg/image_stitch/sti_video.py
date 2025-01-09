from importClass.cam_class import MultiCamNode
import rclpy

def main(args=None):
    rclpy.init(args=args)
    subs = MultiCamNode()
    subs.get_logger().info('MultiCam Stitching node started')

    while rclpy.ok():
        rclpy.spin_once(subs, timeout_sec=0.1)

        images = []
        for cam, info in subs.camera_subscribers.items():
            if not info["queue"].empty():
                images.append(info["queue"].get())

        if len(images) == 3:
            subs.publish_stitched_image(images)

    subs.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
