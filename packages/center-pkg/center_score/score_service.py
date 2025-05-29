import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from center_score.importClass import SensorCallback
from center_score.importClass import RegionLogic
from std_msgs.msg import Int32

class ScoreService(Node):
    def __init__(self, sensor_node):
        super().__init__('score_service')
        self.sensor_node = sensor_node
        self.region_logic = RegionLogic(sensor_node)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.score_pub = self.create_publisher(Int32, '/score', 10)

        self.score = 0
        self.publish_count = 0

    def timer_callback(self):
        total_score = 0
        superstar_score = self.region_logic.check_superstar_pose_in_region()
        sima_score = self.region_logic.check_sima_pose_in_region()
        robot_score = self.region_logic.check_robot_pose_in_region()
        build_score = self.region_logic.check_platform_pose_in_region()
        total_score += sima_score + robot_score + build_score + superstar_score

        self.score = total_score
        self.score_pub.publish(Int32(data=total_score))

        # Log 
        self.publish_count += 1
        if self.publish_count % 10 == 0:
            self.get_logger().info(f'Current Score: {total_score}')
            self.get_logger().info(f'Superstar: {superstar_score}, Sima: {sima_score}, Robot: {robot_score}, Build: {build_score}')

def main(args=None):
    rclpy.init(args=args)

    sensor_node = SensorCallback()
    score_service = ScoreService(sensor_node)

    executor = MultiThreadedExecutor()
    executor.add_node(sensor_node)
    executor.add_node(score_service)

    score_service.get_logger().info('Score service system started.')

    try:
        executor.spin()
    except KeyboardInterrupt:
        score_service.get_logger().info('KeyboardInterrupt, shutting down...')
    finally:
        score_service.get_logger().info('Shutting down nodes...')
        sensor_node.destroy_node()
        score_service.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()
