import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.utilities import perform_substitutions
from ament_index_python.packages import get_package_share_directory

def load_config_and_create_node(context, *args, **kwargs):
   
    # Config File location 
    pipeline_config_path = perform_substitutions(
        context, [LaunchConfiguration('pipeline_config')]
    )
    
    detector_config_path = perform_substitutions(
        context, [LaunchConfiguration('detector_config')]
    )

    # Load Params
    node_parameters = {}
    
    try:
        with open(pipeline_config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # Extract parameters directly from aruco_pipeline section
        if 'aruco_pipeline' in config:
            node_parameters = config['aruco_pipeline'].copy()

        camera_lists = node_parameters.get('camera_lists', ['left', 'mid', 'right'])
        
        print("\n" + "=" * 70)
        print(f"{'ArucoPipeline Launch Configuration'.center(70)}")
        print("=" * 70)

        print_wrapped_line("Config file:", pipeline_config_path, width=70, indent=20)
        print(f"{'Cameras:':<20}{camera_lists}")
        print(f"{'GUI enabled:':<20}{node_parameters.get('gui', False)}")
        print(f"{'Threads:':<20}{node_parameters.get('num_threads', 3)}")
        
        # Print camera topics
        camera_infos = node_parameters.get('camera_info', {})
        if camera_infos:
            print(f"{'Camera Topics:':<20}")
            for camera, topic in camera_infos.items():
                print(f"{' ' * 20}{camera}: {topic}")
        
        # Print pose topics
        blue_topic = node_parameters.get('blue_pose_topic', '/aruco/superstar_pose')
        yellow_topic = node_parameters.get('yellow_pose_topic', '/aruco/sima_pose_array')
        print(f"{'Pose Topics:':<20}")
        print(f"{' ' * 20}Blue: {blue_topic}")
        print(f"{' ' * 20}Yellow: {yellow_topic}")
        
        print("=" * 70)
        
    except Exception as e:
        print(f"Error reading config file {pipeline_config_path}: {e}")
        print("Using default configuration...")
        node_parameters = {}

    # Add detector config path to parameters
    node_parameters['detector_config'] = detector_config_path
    
    # Node
    aruco_node = Node(
        package='aruco_pipeline',
        executable='aruco_pipeline_sima',
        name='aruco_pipeline_sima',
        output='screen',
        parameters=[node_parameters],
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
    package_dir = get_package_share_directory('aruco_pipeline')

    pipeline_config_arg = DeclareLaunchArgument(
        'pipeline_config',
        default_value=os.path.join(
            package_dir,
            'config',
            'sima',
            'pipeline.yaml'
        ),
        description='Path to the aruco pipeline config file'
    )

    detector_file_arg = DeclareLaunchArgument(
        'detector_config',
        default_value=os.path.join(
            package_dir,
            'config',
            'sima',
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
        pipeline_config_arg,
        detector_file_arg,
        camera_count_arg,
        gui_arg,
        aruco_node_launcher
    ])