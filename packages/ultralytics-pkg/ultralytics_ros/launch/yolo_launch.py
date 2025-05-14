import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ultralytics_ros",  
            executable="detect_node", 
            name="region_detector",  
            parameters=["/home/ultralytics/vision-ws/src/ultralytics-ros/ultralytics_ros/config/yolo_params.yaml"],
            # parameters=[{
            #     "platform_poses": "/vision/global_center_poses/platform",
            #     "has_material": "/vision/global_center_poses/has_material"
            # }],
            output="screen",
        ),
        Node(
            package="ultralytics_ros",  
            executable="yolo_all_node", 
            name="yolo_all_node",  
            parameters=["/home/ultralytics/vision-ws/src/ultralytics-ros/ultralytics_ros/config/yolo_params.yaml"],
            # parameters=[{
            #     "model_path": "/home/ultralytics/vision-ws/src/ultralytics-ros/weight/ver7_0507.pt",
            #     "color_topic": "/vision/stitched_image/color/image_raw",
            #     "depth_topic": "/vision/stitched_image/depth/image_raw",
            #     "bbox_topic": "/vision/bounding_boxes",
            #     "platform_pose_topic": "/vision/global_center_poses/platform",
            #     "column_pose_topic": "/vision/global_center_poses/column",
            #     "from_frame_id": "cam_mid_color_optical_frame",
            #     "to_frame_id": "map"
            # }],
            output="screen",
        ),
    ])
