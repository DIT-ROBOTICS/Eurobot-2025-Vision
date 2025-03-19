from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess



def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ultralytics_ros",  
            executable="yolo_node", 
            name="yolo_node",  
            parameters=[{
                "model_path": "/home/ultralytics/vision-ws/src/ultralytics-ros/weight/ver5.pt",
                "color_topic": "/realsense2/cam_mid/color/image_raw",
                "depth_topic": "/realsense2/cam_mid/aligned_depth_to_color/image_raw",
                "bbox_topic": "/detected/bounding_boxes",
                "platform_pose_topic": "detected/global_center_poses/platform",
                "column_pose_topic": "detected/global_center_poses/column",
                "from_frame_id": "cam_mid_color_optical_frame",
                "to_frame_id": "map"
            }],
            output="screen",
        ),
        Node(
            package="ultralytics_ros",  
            executable="detect_node", 
            name="region_detector",  
            output="screen",
        ),
    ])
