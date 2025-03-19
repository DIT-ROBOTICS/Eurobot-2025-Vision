from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    eye = perform_substitutions(context, [LaunchConfiguration('eye')])
    if eye == 'left':
        num = 1
    elif eye == 'mid':
        num = 2
    elif eye == 'right':
        num = 3

    aruco_single_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': LaunchConfiguration('marker_id'),
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': 'cam_' + eye + '_color_optical_frame',
        'marker_frame': LaunchConfiguration('marker_frame'),
        'corner_refinement': LaunchConfiguration('corner_refinement'),
        'detection_mode': LaunchConfiguration('detection_mode'),
    }

    aruco_single = Node(
        package='aruco_ros',
        executable='single',
        parameters=[aruco_single_params],
        remappings=[('/camera_info', f'/realsense{num}/cam_{eye}/color/camera_info'),
                    ('/image', f'/realsense{num}/cam_{eye}/color/image_raw')],
    )

    return [aruco_single]


def generate_launch_description():

    marker_id_arg = DeclareLaunchArgument(
        'marker_id', default_value='6',
        description='Marker ID. '
    )

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.1',
        description='Marker size in m. '
    )

    eye_arg = DeclareLaunchArgument(
        'eye', default_value='left',
        description='Eye. ',
        choices=['left', 'mid' ,'right'],
    )

    marker_frame_arg = DeclareLaunchArgument(
        'marker_frame', default_value='aruco_marker_frame',
        description='Frame in which the marker pose will be refered. '
    )

    reference_frame = DeclareLaunchArgument(
        'reference_frame', default_value='map',
        description='Reference frame. '
        'Leave it empty and the pose will be published wrt param parent_name. '
    )

    corner_refinement_arg = DeclareLaunchArgument(
        'corner_refinement', default_value='LINES',
        description='Corner Refinement. ',
        choices=['SUBPIX','LINES','NONE'],
    )

    dectection_mode_arg = DeclareLaunchArgument(
        'detection_mode', default_value='DM_VIDEO_FAST',
        description='Detection Mode. ',
        choices=['DM_NORMAL','DM_VIDEO_FAST','DM_FAST'],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(marker_id_arg)
    ld.add_action(marker_size_arg)
    ld.add_action(eye_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(reference_frame)
    ld.add_action(corner_refinement_arg)
    ld.add_action(dectection_mode_arg)

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
