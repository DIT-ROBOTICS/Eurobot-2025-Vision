from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([
        # 靜態 TF 變換節點
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
                '0', '0', '0',  # 平移 (x, y, z)
                '0', '0', '0', '1',  # 四元數旋轉 (x, y, z, w)
                'camera_link', 'world'  # 父座標框架和子座標框架
            ],
            output='screen'
        ),

        # YOLO 節點
        Node(
            package='your_package_name',  # 替換為實際的 ROS 2 package 名稱
            executable='yolo_node',      # 替換為可執行檔的名稱
            name='yolo_node',
            output='screen'
        ),

        # TF 節點
        Node(
            package='your_package_name',  # 替換為實際的 ROS 2 package 名稱
            executable='tf_node',        # 替換為可執行檔的名稱
            name='tf_node',
            output='screen'
        )
    ])
