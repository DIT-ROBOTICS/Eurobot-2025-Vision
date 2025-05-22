import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        os.path.dirname(__file__),
        '..',
        'config',
        'config.yaml'
    )

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    aruco_node = Node(
        package='aruco_pipeline',
        executable='aruco_pipeline',
        name='aruco_pipeline',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/aruco/robot_pose', config['aruco_pipeline']['ros__parameters']['robot_pose_topic']),
            ('/aruco/rival_pose', config['aruco_pipeline']['ros__parameters']['rival_pose_topic'])
        ],
        respawn=True,
        respawn_delay=5.0,
    )

    return LaunchDescription([
        aruco_node
    ])
