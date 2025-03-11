from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        # 靜態 TF 變換節點
        
        # YOLO 節點
        Node(
            package='ultralytics_ros',  # 替換為實際的 ROS 2 package 名稱
            executable='yolo_node',     
            name='yolo_node',
            output='screen'
        ),

        # TF 節點
        Node(
            package='ultralytics_ros',  # 替換為實際的 ROS 2 package 名稱
            executable='tf_node',     
            name='tf_node',
            output='screen'
        )
    ])
