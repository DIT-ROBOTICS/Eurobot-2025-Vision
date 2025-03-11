from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
import launch_ros.actions

def launch_map_transform_publisher_node(context: LaunchContext):
    node = launch_ros.actions.Node(
        name='map_transform_publisher',
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            '1.630', '1.080', '1.785', '0', '-1', '0', '0',
            'map',
            'ceiling'
        ]
    )
    return [node]



def launch_setup(context, *args, **kwargs):

    robot_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': 6,
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': 'ceiling',
        'marker_frame': LaunchConfiguration('marker_frame'),
        'corner_refinement': LaunchConfiguration('corner_refinement'),
        'detection_mode': LaunchConfiguration('detection_mode'),
    }

    rival_params = {
        'image_is_rectified': True,
        'marker_size': LaunchConfiguration('marker_size'),
        'marker_id': 21,
        'reference_frame': LaunchConfiguration('reference_frame'),
        'camera_frame': 'ceiling',
        'marker_frame': LaunchConfiguration('marker_frame'),
        'corner_refinement': LaunchConfiguration('corner_refinement'),
        'detection_mode': LaunchConfiguration('detection_mode'),
    }

    robot = Node(
        package='aruco_ros',
        executable='single',
        name = 'ceiling_robot',
        parameters=[robot_params],
        remappings=[('/camera_info', '/ceiling_cam/camera_info'),
                    ('/image', '/ceiling_cam/image_rect')],
    )

    rival = Node(
        package='aruco_ros',
        executable='single',
        name = 'ceiling_rival',
        parameters=[rival_params],
        remappings=[('/camera_info', '/ceiling_cam/camera_info'),
                    ('/image', '/ceiling_cam/image_rect')],
    )

    return [robot,rival]


def generate_launch_description():

    marker_size_arg = DeclareLaunchArgument(
        'marker_size', default_value='0.1',
        description='Marker size in m. '
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

    ld.add_action(marker_size_arg)
    ld.add_action(marker_frame_arg)
    ld.add_action(reference_frame)
    ld.add_action(corner_refinement_arg)
    ld.add_action(dectection_mode_arg)

    ld.add_action(OpaqueFunction(function=launch_map_transform_publisher_node))
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
