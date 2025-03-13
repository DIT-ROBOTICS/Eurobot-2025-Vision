import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

class TFNode(Node):
    def __init__(self):
        super().__init__('tf_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_sub_platform = self.create_subscription(PoseArray, '/detected/cam_pose_array/platform', self.pose_callback_platform, 10)
        self.pose_sub_column = self.create_subscription(PoseArray, '/detected/cam_pose_array/column', self.pose_callback_column, 10)
        self.global_pub_platform = self.create_publisher(PoseArray, '/detected/global_center_poses/platform', 10)
        self.global_pub_column = self.create_publisher(PoseArray, '/detected/global_center_poses/column', 10)
        self.get_logger().info("TF Node initialized and ready.")

    def pose_callback_platform(self, msg):
        transformed_pose_array = PoseArray()
        transformed_pose_array.header.frame_id = "map"
        transformed_pose_array.header.stamp = self.get_clock().now().to_msg()

        for pose in msg.poses:
            try:
                transformed_pose = self.transform_pose(msg.header.frame_id, "map", pose)
                if transformed_pose:
                    transformed_pose_array.poses.append(transformed_pose)
            except Exception as e:
                self.get_logger().error(f"Transform failed: {str(e)}")

        print(f"transformed array has {len(transformed_pose_array.poses)} {'object' if len(transformed_pose_array.poses) == 1 else 'objects'}")

        final_pose_array = PoseArray()
        final_pose_array.header.frame_id = "map"
        final_pose_array.header.stamp = self.get_clock().now().to_msg()

        for i in range(0, len(transformed_pose_array.poses) - 1, 3):
            finalpose = Pose()
            finalpose.position.x = (transformed_pose_array.poses[i].position.x + transformed_pose_array.poses[i+1].position.x) / 2
            finalpose.position.y = (transformed_pose_array.poses[i].position.y + transformed_pose_array.poses[i+1].position.y) / 2
            finalpose.position.z = (transformed_pose_array.poses[i+2].position.z )
            length = (transformed_pose_array.poses[i].position.x - transformed_pose_array.poses[i+1].position.x)*100
            height = (transformed_pose_array.poses[i].position.y - transformed_pose_array.poses[i+1].position.y)*100

            print(f"length = {length}")
            print(f"height = {height}")

            if(length<=10):
                length=10.00001
            elif(length>41.231056):
                length=41.231056
            if(height<=10):
                height=10.00001
            elif(height>41.231056):
                height=41.231056

            if (length>=height):
                angle = math.acos(height/41.231056)+1.3258176
            else:
                angle = math.acos(length/41.231056)+0.2449786
            print(f"angle = {angle}")
            finalpose.orientation.x = 0.0
            finalpose.orientation.y = 0.0
            finalpose.orientation.z = math.sin(angle / 2)
            finalpose.orientation.w = math.cos(angle / 2)
            final_pose_array.poses.append(finalpose)  
            print(f"pose_array has{len(final_pose_array.poses)} object")
        if len(final_pose_array.poses) > 0:
            self.global_pub_platform.publish(final_pose_array)  



    def pose_callback_column(self, msg):
        transformed_pose_array = PoseArray()
        transformed_pose_array.header.frame_id = "map"
        transformed_pose_array.header.stamp = self.get_clock().now().to_msg()

        for pose in msg.poses:
            try:
                transformed_pose = self.transform_pose(msg.header.frame_id, "map", pose)
                if transformed_pose:
                    transformed_pose_array.poses.append(transformed_pose)

            except Exception as e:
                self.get_logger().error(f"Transform failed: {str(e)}")

        final_pose_array = PoseArray()
        final_pose_array.header.frame_id = "map"
        final_pose_array.header.stamp = self.get_clock().now().to_msg()

        for i in range(0, len(transformed_pose_array.poses) - 1, 3):
            finalpose = Pose()
            finalpose.position.x = (transformed_pose_array.poses[i].position.x + transformed_pose_array.poses[i+1].position.x) / 2
            finalpose.position.y = (transformed_pose_array.poses[i].position.y + transformed_pose_array.poses[i+1].position.y) / 2
            finalpose.position.z = (transformed_pose_array.poses[i+2].position.z)  
            finalpose.orientation.x = 0.0
            finalpose.orientation.y = 0.0
            finalpose.orientation.z = 0.0
            finalpose.orientation.w = 1.0
            final_pose_array.poses.append(finalpose)  
        if len(final_pose_array.poses) > 0:
            self.global_pub_column.publish(final_pose_array) 

    def transform_pose(self, from_frame, to_frame, pose):
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = from_frame
            pose_stamped.header.stamp = rclpy.time.Time().to_msg()
            pose_stamped.pose = pose

            self.tf_buffer.can_transform(to_frame, from_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            transformed_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, timeout=rclpy.duration.Duration(seconds=1.0))
            
            return transformed_pose_stamped.pose
        except Exception as e:
            self.get_logger().error(f"Pose transform failed: {str(e)}")
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
