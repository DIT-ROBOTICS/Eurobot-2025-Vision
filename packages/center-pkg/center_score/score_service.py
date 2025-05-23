import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from center_score.importClass import SensorCallback
from center_score.importClass import RegionLogic
from std_msgs.msg import Float32

class ScoreService(Node):
    def __init__(self, sensor_node):
        super().__init__('score_service')
        self.sensor_node = sensor_node
        self.region_logic = RegionLogic(sensor_node)

        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.score_timer = self.create_timer(100, self.score_timer_callback)
        self.score_pub = self.create_publisher(Float32, 'estimated_score', 1)

        self.score = 0

    def timer_callback(self):
        # total_score = 0
        superstar_score = self.region_logic.check_superstar_pose_in_region()
        sima_score = self.region_logic.check_sima_pose_in_region()
        robot_score = self.region_logic.check_robot_pose_in_region()
        build_score = self.region_logic.check_platform_pose_in_region()
        self.score += sima_score + robot_score + build_score + superstar_score

        self.get_logger().info(f'Current Score: {self.score}')

    def score_timer_callback(self):
        self.score_pub.publish(self.score)



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
