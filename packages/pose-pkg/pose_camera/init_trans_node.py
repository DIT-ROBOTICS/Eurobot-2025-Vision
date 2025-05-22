import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from pose_camera.importClass import SensorCallback

class PoseCamera(Node):
    def __init__(self, sensor_node):
        super().__init__('pose_camera_node')
        self.sensor_node = sensor_node

        self.timer_period = 0.1 # 10Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # def timer_callback(self):
    #     superstar_score = self.region_logic.check_superstar_pose_in_region()
    #     sima_score = self.region_logic.check_sima_pose_in_region()
    #     robot_score = self.region_logic.check_robot_pose_in_region()
    #     build_score = self.region_logic.check_platform_pose_in_region()
    #     total_score += sima_score + robot_score + build_score + superstar_score

def main(args=None):
    rclpy.init(args=args)

    sensor_node = SensorCallback()
    pose_camera = PoseCamera(sensor_node)

    executor = MultiThreadedExecutor()
    executor.add_node(sensor_node)
    executor.add_node(pose_camera)

    pose_camera.get_logger().info('Camera Pose Inferencing node started.')

    try:
        executor.spin()
    except KeyboardInterrupt:
        pose_camera.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        pose_camera.get_logger().info('Shutting down nodes...')
        sensor_node.destroy_node()
        pose_camera.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
