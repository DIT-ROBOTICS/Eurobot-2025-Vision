import launch
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='camera_ros', 
            executable='image_stitcher_node', 
            name='image_stitcher_color',
            output='screen',
            parameters=[{
                'left_topic': '/realsense1/cam_left/color/image_raw',
                'mid_topic': '/realsense2/cam_mid/color/image_raw',
                'right_topic': '/realsense3/cam_right/color/image_raw',
                'stitched_topic': '/realsense/stitched_image/color/image_raw',
                'image_shape': '360x640x30',
                'encoding': 'bgr8',
                'data_type': 'uint8'
            }],
        ),

        launch_ros.actions.Node(
            package='camera_ros',
            executable='image_stitcher_node',
            name='image_stitcher_depth',
            output='screen',
            parameters=[{
                'left_topic': '/realsense1/cam_left/aligned_depth_to_color/image_raw',
                'mid_topic': '/realsense2/cam_mid/aligned_depth_to_color/image_raw',
                'right_topic': '/realsense3/cam_right/aligned_depth_to_color/image_raw',
                'stitched_topic': '/realsense/stitched_image/depth/image_raw',
                'image_shape': '360x640',
                'encoding': '16UC1',
                'data_type': 'uint16'
            }],
        ),
    ])
