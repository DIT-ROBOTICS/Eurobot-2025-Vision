import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.utilities import perform_substitutions

def load_config_and_create_node(context, *args, **kwargs):
   
    # Config File location 
    config_file_path = perform_substitutions(
        context, [LaunchConfiguration('config_file')]
    )
    
    detector_config_path = perform_substitutions(
        context, [LaunchConfiguration('detector_config')]
    )

    # Load Params
    remappings = []
    node_params = {}
    
    try:
        with open(config_file_path, 'r') as f:
            config = yaml.safe_load(f)
        
        params = config.get('aruco_pipeline', {}).get('ros__parameters', {})

        camera_lists = params.get('camera_lists', ['left', 'mid', 'right'])
        
        print("\n" + "=" * 70)
        print(f"{'ArucoPipeline Launch Configuration'.center(70)}")
        print("=" * 70)

        print_wrapped_line("Config file:", config_file_path, width=70, indent=20)
        print(f"{'Cameras:':<20}{camera_lists}")
        print(f"{'GUI enabled:':<20}{params.get('gui', False)}")
        print(f"{'Threads:':<20}{params.get('num_threads', 3)}")
        
        # Pose topic remap 
        blue_pose_topic = params.get('blue_pose_topic', '/aruco/blue_pose')
        yellow_pose_topic = params.get('yellow_pose_topic', '/aruco/yellow_pose')
        
        if blue_pose_topic != '/aruco/blue_pose':
            remappings.append(('/aruco/blue_pose', blue_pose_topic))
        
        if yellow_pose_topic != '/aruco/yellow_pose':
            remappings.append(('/aruco/yellow_pose', yellow_pose_topic))
        
        # Dynamic Remapping
        for cam_name in camera_lists:
            # Image Remap
            if 'image_remapping' in params and cam_name in params['image_remapping']:
                original_topic = params['image_sub'][cam_name]
                new_topic = params['image_remapping'][cam_name]
                remappings.append((original_topic, new_topic))
                print(f"Image remapping [{cam_name}]: {original_topic} → {new_topic}")
            
            # Camera Info Remap
            if 'camera_info_remapping' in params and cam_name in params['camera_info_remapping']:
                original_topic = params['camera_info'][cam_name]
                new_topic = params['camera_info_remapping'][cam_name]
                remappings.append((original_topic, new_topic))
                print(f"CameraInfo remapping [{cam_name}]: {original_topic} → {new_topic}")
        
        if remappings:
            print(f"{'Remappings:':<20}{len(remappings)} active")
            for old, new in remappings:
                print(f"{'':<22}→ {old}  →  {new}")
        else:
            print(f"{'Remappings:':<20}None")

        print("=" * 70)
        
    except Exception as e:
        print(f"Error reading config file {config_file_path}: {e}")
        print("Using default configuration...")

    # Node
    aruco_node = Node(
        package='aruco_pipeline',
        executable='aruco_pipeline',
        name='aruco_pipeline',
        output='screen',
        parameters=[
            config_file_path,
            {'detector_config': detector_config_path}
        ],
        remappings=remappings,
        respawn=True,
        respawn_delay=5.0,
    )
    
    return [aruco_node]

def print_wrapped_line(label, text, width=70, indent=20):
    import textwrap
    print(f"{label:<{indent}}", end='')
    wrapped_lines = textwrap.wrap(text, width=width - indent)
    if wrapped_lines:
        print(wrapped_lines[0])
        for line in wrapped_lines[1:]:
            print(' ' * indent + line)
    else:
        print()

def generate_launch_description():
    
    # Launch Params
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'pipeline.yaml'
        ),
        description='Path to the aruco pipeline config file'
    )

    detector_file_arg = DeclareLaunchArgument(
        'detector_config',
        default_value=os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'detector.yaml'
        ),
        description='Path to the aruco detector config file'
    )
    
    camera_count_arg = DeclareLaunchArgument(
        'camera_count',
        default_value='auto',
        description='Number of cameras (auto-detect from config file)'
    )
    
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='auto',
        description='Enable GUI (auto-detect from config file, or true/false)'
    )
    
    aruco_node_launcher = OpaqueFunction(
        function=load_config_and_create_node
    )
    
    return LaunchDescription([
        config_file_arg,
        detector_file_arg,
        camera_count_arg,
        gui_arg,
        aruco_node_launcher
    ])
