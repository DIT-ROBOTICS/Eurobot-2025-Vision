from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import math
import rclpy
import numpy as np
class PoseTransformer:
    def __init__(self,tf_buffer,from_frame_id, to_frame_id,f_x, f_y, c_x, c_y,source_points, target_points):   
        self.f_x = f_x
        self.f_y = f_y
        self.c_x = c_x
        self.c_y = c_y
        self.from_frame_id = from_frame_id
        self.to_frame_id = to_frame_id
        self.tf_buffer = tf_buffer
        source_points = np.array(source_points)
        target_points = np.array(target_points)
        # source_points = np.array([
        #     (1.87, 1.06),
        #     (2.20, 0.48),
        #     (0.87, 0.35),
        #     (1.12, 1.027),
        #     (0.19, 0.42),
        #     (0.19, 1.32),
        #     (0.85, 1.72)
            
        # ])
        # target_points = np.array([
        #     (1.895, 0.95),
        #     (2.22, 0.25),
        #     (0.78, 0.25),
        #     (1.095, 0.95),
        #     (0.075, 0.395),
        #     (0.075, 1.32),
        #     (0.82, 1.725)
        # ])
        N = len(source_points)
        A = np.hstack([source_points, np.ones((N, 1))])
        B = target_points  
        X, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
        self.m1 = X[:2].T  
        self.m2 = X[2]            

    def transform_pose(self, pose):
        try:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.from_frame_id
            pose_stamped.header.stamp = rclpy.time.Time().to_msg()
            pose_stamped.pose = pose
            transformed = self.tf_buffer.transform(pose_stamped, self.to_frame_id, timeout=rclpy.duration.Duration(seconds=3.0))
            # print(f"Before calibration: {transformed.pose.position.x}, {transformed.pose.position.y}, {transformed.pose.position.z}")
            transformed.pose = self.calibration(transformed.pose)
            # print(f"After calibration: {transformed.pose.position.x}, {transformed.pose.position.y}, {transformed.pose.position.z}")
            return transformed.pose
        except Exception as e:
            print(f"TF transform failed: {str(e)}")
            return None
        
    def switch_to_cam_pose(self, x, y, z):
        pose = Pose()
        pose.position.y = (z * (x - self.c_x) / self.f_x) / 1000 
        pose.position.x = -(z * (y - self.c_y) / self.f_y) / 1000
        pose.position.z = z / 1000
        pose.orientation.w = 1.0
        return pose

    def calibration(self, pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        pt = np.array([x, y])
        corrected = self.m1 @ pt + self.m2
        pose.position.x = corrected[0]
        pose.position.y = corrected[1]
        return pose