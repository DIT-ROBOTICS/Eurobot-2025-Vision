from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.utilities import perform_substitutions


def launch_setup(context, *args, **kwargs):

    position_processor = Node(
        package='aruco_ros',
        executable='position_processor',
        parameters=[]
    )
    
    sima1 = Node(
        package='aruco_ros',
        namespace='sima1',
        executable='position_processor',
        remappings=[
            ('/left/single/pose', '/sima1/left/sima/pose'),
            ('/mid/single/pose', '/sima1/mid/sima/pose'),
            ('/right/single/pose', '/sima1/right/sima/pose'),
            ('/average_pose', '/sima1/average_pose')
        ],
        parameters=[]
    )
    
    sima2 = Node(
        package='aruco_ros',
        namespace='sima2',
        executable='position_processor',
        remappings=[
            ('/left/single/pose', '/sima2/left/sima/pose'),
            ('/mid/single/pose', '/sima2/mid/sima/pose'),
            ('/right/single/pose', '/sima2/right/sima/pose'),
            ('/average_pose', '/sima2/average_pose')
        ],
        parameters=[]
    )
    
    sima3 = Node(
        package='aruco_ros',
        namespace='sima3',
        executable='position_processor',
        remappings=[
            ('/left/single/pose', '/sima3/left/sima/pose'),
            ('/mid/single/pose', '/sima3/mid/sima/pose'),
            ('/right/single/pose', '/sima3/right/sima/pose'),
            ('/average_pose', '/sima3/average_pose')
        ],
        parameters=[]
    )
    
    sima4 = Node(
        package='aruco_ros',
        namespace='sima4',
        executable='position_processor',
        remappings=[
            ('/left/single/pose', '/sima4/left/sima/pose'),
            ('/mid/single/pose', '/sima4/mid/sima/pose'),
            ('/right/single/pose', '/sima4/right/sima/pose'),
            ('/average_pose', '/sima4/average_pose')
        ],
        parameters=[]
    )

    sima_pose = Node(
        package='aruco_ros',
        executable='sima_pose_array',
        parameters=[]
    )

    return [position_processor, sima1, sima2, sima3, sima4, sima_pose]


def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld