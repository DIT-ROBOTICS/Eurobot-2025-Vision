from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math
import rclpy

class PoseTransformer:
    def __init__(self,tf_buffer,from_frame_id, to_frame_id):    
        self.from_frame_id = from_frame_id
        self.to_frame_id = to_frame_id
        self.tf_buffer = tf_buffer
    def transform_pose(self, pose):
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.from_frame_id
            pose_stamped.header.stamp = rclpy.time.Time().to_msg()
            pose_stamped.pose = pose

            transformed = self.tf_buffer.transform(pose_stamped, self.to_frame_id, timeout=rclpy.duration.Duration(seconds=2.0))
            return transformed.pose
        except Exception as e:
            print(f"TF transform failed: {str(e)}")
            return None
    def switch_to_cam_pose(self, x, y, z):
        f_x = 476.4030
        f_y = 467.9718
        c_x = 533.1214
        c_y = 291.4719
        pose = Pose()
        pose.position.y = (z * (x - c_x) / f_x) / 1000 
        pose.position.x = -(z * (y - c_y) / f_y) / 1000
        pose.position.z = z / 1000
        pose.orientation.w = 1.0
        return pose

 