from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    side = perform_substitutions(context, [LaunchConfiguration('side')])

    if side == 'left':
        num = 1
    elif side == 'mid':
        num = 2
    elif side == 'right':
        num = 3

    aruco_marker_publisher_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': f'cam_{side}_color_optical_frame',
    }

    aruco_marker_publisher = Node(
        package='aruco_ros',
        executable='marker_publisher',
        parameters=[aruco_marker_publisher_params],
        remappings=[('/camera_info', f'/realsense{num}/cam_{side}/color/camera_info'),
                    ('/image', f'/realsense{num}/cam_{side}/color/image_raw')],
    )

    return [aruco_marker_publisher]


def generate_launch_description():

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.1',
        description='Marker size in m. '
    )

    # side_arg = DeclareLaunchArgument(
    #     'side', default_value='left',
    #     description='Side. ',
    # )

    reference_frame = DeclareLaunchArgument(
        'reference_frame', default_value='map'
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(marker_size_arg)
    #ld.add_action(side_arg)
    ld.add_action(reference_frame)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
