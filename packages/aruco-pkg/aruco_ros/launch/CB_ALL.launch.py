from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    package_name = 'aruco_ros'

    # Include CB_pose.launch.py
    cb_pose_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch','CB_pose.launch.py')
            )
    )

    # Include multi_sima_publisher.launch.py
    multi_sima_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'multi_sima_publisher.launch.py'))
    )

    # Include multicam_publisher.launch.py
    multicam_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package_name), 'launch', 'multicam_publisher.launch.py'))
    )

    # Add the launch descriptions to the launch description
    ld.add_action(cb_pose_launch)
    ld.add_action(multi_sima_publisher_launch)
    ld.add_action(multicam_publisher_launch)

    return ld