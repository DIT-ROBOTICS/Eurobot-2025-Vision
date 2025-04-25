from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Define namespaces and corresponding marker IDs
    camera_configs = [
        {'namespace': 'sima1', 'marker_id': '1'},
        {'namespace': 'sima2', 'marker_id': '2'},
        {'namespace': 'sima3', 'marker_id': '5'},
        {'namespace': 'sima4', 'marker_id': '4'},
    ]

    # IncludeLaunchDescription for each namespace
    package_name = 'aruco_ros'  # Replace with your package name
    launch_file_name = 'sima_publisher.launch.py'  # The launch file to be included

    # Create a list to store included launches
    include_launches = []

    for config in camera_configs:
        # Define a group for each namespace to ensure proper namespaces if needed
        include_launch = GroupAction([
            PushRosNamespace(config['namespace']),  # Use namespaces based on the list
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(package_name), 'launch', launch_file_name
                    )
                ),
                launch_arguments={
                    'marker_id': config['marker_id'],
                }.items(),
            ),
        ])
        include_launches.append(include_launch)

    # Build the final LaunchDescription
    ld = LaunchDescription(include_launches)

    return ld