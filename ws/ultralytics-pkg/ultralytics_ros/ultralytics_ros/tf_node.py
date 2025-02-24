# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseArray, Pose, PointStamped
# from tf2_ros import Buffer, TransformListener
# import tf2_geometry_msgs
    
# class TFNode(Node):
#     def __init__(self):
#         super().__init__('tf_node')

#         # TF 緩衝器和監聽器
#         self.tf_buffer = Buffer()
#         self.tf_listener = TransformListener(self.tf_buffer, self)

#         # 訂閱中心點座標
#         self.center_sub_platform = self.create_subscription(PointStamped, '/detected/cam_center_points/platform', self.center_callback_platform, 10)
#         self.center_sub_column = self.create_subscription(PointStamped, '/detected/cam_center_points/column', self.center_callback_column, 10)

#         # 發布全局座標 (PoseArray)
#         self.global_pub_platform = self.create_publisher(PoseArray, '/detected/global_center_poses/platform', 10)
#         self.global_pub_column = self.create_publisher(PoseArray, '/detected/global_center_poses/column', 10)
#         self.platform_poses = PoseArray()
#         self.column_poses = PoseArray()

#         self.get_logger().info("TF Node initialized and ready.")

#         self.timer = self.create_timer(0.5, self.publish_pose_arrays)
        

#     def center_callback_platform(self, msg):
#         self.process_point(msg, self.platform_poses)

#     def center_callback_column(self, msg):
#         self.process_point(msg, self.column_poses)

#     def process_point(self, msg, pose_array):
#         try:
#             # 轉換中心點座標到 map frame
#             transformed_point = self.tf_buffer.transform(
#                 msg, target_frame="map", timeout=rclpy.duration.Duration(seconds=1.0)
#             )

#             pose = Pose()
#             pose.position.x = transformed_point.point.x
#             pose.position.y = transformed_point.point.y
#             pose.position.z = transformed_point.point.z

#             pose.orientation.w = 1.0  #假設無轉

#             # 添加到 PoseArray
#             pose_array.poses.append(pose)
#             pose_array.header.frame_id = "map"
#             pose_array.header.stamp = self.get_clock().now().to_msg()

#             self.get_logger().info(f"Added Global Pose: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")

#         except Exception as e:
#             self.get_logger().error(f"Transform failed: {str(e)}")
    
#     def publish_pose_arrays(self):
#         if self.platform_poses.poses:
#             self.global_pub_platform.publish(self.platform_poses)
#             self.platform_poses.poses.clear()  
        
#         if self.column_poses.poses:
#             self.global_pub_column.publish(self.column_poses)
#             self.column_poses.poses.clear()

# def main(args=None):
#     rclpy.init(args=args)
#     tf_node = TFNode()

#     try:
#         rclpy.spin(tf_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         tf_node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class TFNode(Node):
    def __init__(self):
        super().__init__('tf_node')

        # TF 緩衝器和監聽器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 訂閱來自相機座標系的 PoseArray
        self.pose_sub_platform = self.create_subscription(PoseArray, '/detected/cam_pose_array/platform', self.pose_callback_platform, 10)
        self.pose_sub_column = self.create_subscription(PoseArray, '/detected/cam_pose_array/column', self.pose_callback_column, 10)

        # 發布全局座標的 PoseArray
        self.global_pub_platform = self.create_publisher(PoseArray, '/detected/global_center_poses/platform', 10)
        self.global_pub_column = self.create_publisher(PoseArray, '/detected/global_center_poses/column', 10)

        self.get_logger().info("TF Node initialized and ready.")

    def pose_callback_platform(self, msg):
        transformed_pose_array = PoseArray()
        transformed_pose_array.header.frame_id = "map"
        transformed_pose_array.header.stamp = self.get_clock().now().to_msg()

        for pose in msg.poses:
            try:
                # 將 Pose 轉換到 map frame
                transformed_pose = self.transform_pose(msg.header.frame_id, "map", pose)
                if transformed_pose:
                    transformed_pose_array.poses.append(transformed_pose)
            except Exception as e:
                self.get_logger().error(f"Transform failed: {str(e)}")

        print(f"transformed array has {len(transformed_pose_array.poses)} {'object' if len(transformed_pose_array.poses) == 1 else 'objects'}")

        final_pose_array = PoseArray()
        final_pose_array.header.frame_id = "map"
        final_pose_array.header.stamp = self.get_clock().now().to_msg()

        for i in range(0, len(transformed_pose_array.poses) - 1, 2):
            finalpose = Pose()
            finalpose.position.x = (transformed_pose_array.poses[i].position.x + transformed_pose_array.poses[i+1].position.x) / 2
            finalpose.position.y = (transformed_pose_array.poses[i].position.y + transformed_pose_array.poses[i+1].position.y) / 2
            finalpose.position.z = (transformed_pose_array.poses[i].position.z + transformed_pose_array.poses[i+1].position.z) / 2  

            finalpose.orientation = transformed_pose_array.poses[i].orientation
            final_pose_array.poses.append(finalpose)  
        if len(final_pose_array.poses) > 0:
            self.global_pub_platform.publish(final_pose_array)  



    def pose_callback_column(self, msg):
        transformed_pose_array = PoseArray()
        transformed_pose_array.header.frame_id = "map"
        transformed_pose_array.header.stamp = self.get_clock().now().to_msg()

        for pose in msg.poses:
            try:
                # 將 Pose 轉換到 map frame
                transformed_pose = self.transform_pose(msg.header.frame_id, "map", pose)
                if transformed_pose:
                    transformed_pose_array.poses.append(transformed_pose)

            except Exception as e:
                self.get_logger().error(f"Transform failed: {str(e)}")

        final_pose_array = PoseArray()
        final_pose_array.header.frame_id = "map"
        final_pose_array.header.stamp = self.get_clock().now().to_msg()

        for i in range(0, len(transformed_pose_array.poses) - 1, 2):
            finalpose = Pose()
            finalpose.position.x = (transformed_pose_array.poses[i].position.x + transformed_pose_array.poses[i+1].position.x) / 2
            finalpose.position.y = (transformed_pose_array.poses[i].position.y + transformed_pose_array.poses[i+1].position.y) / 2
            finalpose.position.z = (transformed_pose_array.poses[i].position.z + transformed_pose_array.poses[i+1].position.z) / 2  

            finalpose.orientation = transformed_pose_array.poses[i].orientation
            final_pose_array.poses.append(finalpose)  
        if len(final_pose_array.poses) > 0:
            self.global_pub_column.publish(final_pose_array) 

        

    def transform_pose(self, from_frame, to_frame, pose):
        try:
            transform = self.tf_buffer.lookup_transform(to_frame, from_frame, rclpy.time.Time(),rclpy.duration.Duration(seconds=1.0))
            
            transformed_pose = Pose()
            transformed_pose.position.x = (
                pose.position.x + transform.transform.translation.x
            )
            transformed_pose.position.y = (
                pose.position.y + transform.transform.translation.y
            )
            transformed_pose.position.z = (
                pose.position.z + transform.transform.translation.z
            )

            transformed_pose.orientation = pose.orientation  # 保持原始方向（假設無旋轉）

            return transformed_pose

        except Exception as e:
            self.get_logger().error(f"Transform lookup failed: {str(e)}")
            return None

def main(args=None):
    rclpy.init(args=args)
    tf_node = TFNode()

    try:
        rclpy.spin(tf_node)
    except KeyboardInterrupt:
        pass
    finally:
        tf_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
