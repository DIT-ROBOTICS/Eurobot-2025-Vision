from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'aruco_pipeline', 'aruco_pipeline_robot.launch.py'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'launch', 'aruco_pipeline', 'aruco_pipeline_sima.launch.py'],
            output='screen'
        )
    ])
