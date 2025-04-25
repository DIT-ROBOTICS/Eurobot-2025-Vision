from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math

class PoseTransformer:
    def __init__(self):
        self.declare_parameter("from_frame_id", "cam_mid_color_optical_frame")
        self.declare_parameter("to_frame_id", "map")
        self.from_frame_id = self.get_parameter("from_frame_id").value
        self.to_frame_id = self.get_parameter("to_frame_id").value
        self.tf_buffer = tf_buffer
        self.from_frame = from_frame
        self.to_frame = to_frame


    def transform_pose(self, pose):
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.from_frame
            pose_stamped.header.stamp = rclpy.time.Time().to_msg()
            pose_stamped.pose = pose

            transformed = self.tf_buffer.transform(pose_stamped, self.to_frame, timeout=rclpy.duration.Duration(seconds=2.0))
            return transformed.pose
        except Exception as e:
            (f"TF transform failed: {str(e)}")
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