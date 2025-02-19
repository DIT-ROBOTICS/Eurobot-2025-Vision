from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare global arguments
    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.1',
        description='Marker size in meters.'
    )

    reference_frame_arg = DeclareLaunchArgument(
        'reference_frame', default_value='map',
        description='Reference frame for the markers.'
    )

    # Define sides
    sides = ['left', 'right', 'mid']

    # IncludeLaunchDescription for each side
    package_name = 'aruco_ros'  # Replace with your package name
    launch_file_name = 'single.launch.py'  # Original launch file name

    # Create a list to store included launches
    include_launches = []

    for side in sides:
        # Define a group for each side to ensure proper namespaces if needed
        include_launch = GroupAction([
            PushRosNamespace(side),  # Optional: Use namespaces based on the side
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(package_name), 'launch', launch_file_name
                    )
                ),
                launch_arguments={
                    'eye': TextSubstitution(text=side),
                    'marker_size': LaunchConfiguration('marker_size'),
                    'reference_frame': LaunchConfiguration('reference_frame'),
                }.items(),
            ),
        ])
        include_launches.append(include_launch)

    # Build the final LaunchDescription
    ld = LaunchDescription([
        marker_size_arg,
        reference_frame_arg,
        *include_launches,  # Add all grouped launches
    ])

    return ld
