import launch
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer

def generate_launch_description():
    return launch.LaunchDescription([
        ComposableNodeContainer(
            name='rtsp_cam_container',
            namespace='ceiling_cam',
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='rtsp_ros',
                    plugin='rtsp_ros::CamNodePublisher',
                    name='rtsp_cam_node',
                    parameters=[{
                        'rtsp_url': 'rtsp://192.168.50.60:8554/camera',
                        'camera_name': 'ceiling_cam',
                        'camera_info_file': 'package://rtsp_ros/config/ceiling_cam_calibration.yaml'
                    }],
                    remappings=[
                        ('image_raw', '/ceiling_cam/image_raw'),
                        ('camera_info', '/ceiling_cam/camera_info')
                    ]
                ),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_node',
                    remappings=[
                        ('image', '/ceiling_cam/image_raw'),
                        ('camera_info', '/ceiling_cam/camera_info'),
                        ('image_rect', '/ceiling_cam/image_rect')
                    ]
                )
            ],
            output='screen',
        )
    ])
