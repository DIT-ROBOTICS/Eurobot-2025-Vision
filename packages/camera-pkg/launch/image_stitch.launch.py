import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='image_stitcher_node',
            name='image_stitcher_color',
            output='screen',
            parameters=[{
                'left_topic': '/vision/cam_left/color/image_raw',
                'mid_topic': '/vision/cam_mid/color/image_raw',
                'right_topic': '/vision/cam_right/color/image_raw',
                'stitched_topic': '/vision/stitched_image/color/image_raw',
                'target_image_shape': '360x640',
            }],
        ),

        Node(
            package='camera_ros',
            executable='image_stitcher_node',
            name='image_stitcher_depth',
            output='screen',
            parameters=[{
                'left_topic': '/vision/cam_left/aligned_depth_to_color/image_raw',
                'mid_topic': '/vision/cam_mid/aligned_depth_to_color/image_raw',
                'right_topic': '/vision/cam_right/aligned_depth_to_color/image_raw',
                'stitched_topic': '/vision/stitched_image/depth/image_raw',
                'target_image_shape': '360x640',
            }],
        ),
    ])
