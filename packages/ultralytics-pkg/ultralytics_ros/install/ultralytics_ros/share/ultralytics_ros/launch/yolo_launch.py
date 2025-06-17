import rclpy
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ultralytics_ros",  
            executable="detect_node", 
            name="detect_node",  
            parameters=["/home/ultralytics/vision-ws/src/ultralytics-ros/ultralytics_ros/config/yolo_params.yaml"],
            output="screen",
        ),
        Node(
            package="ultralytics_ros",  
            executable="yolo_node_nopub", 
            name="yolo_node_nopub",  
            parameters=["/home/ultralytics/vision-ws/src/ultralytics-ros/ultralytics_ros/config/yolo_params.yaml"],
            output="screen",
        ),
    ])
