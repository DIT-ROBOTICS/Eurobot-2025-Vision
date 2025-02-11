# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# DESCRIPTION #
# ----------- #
# Use this launch file to launch 2 devices.
# The Parameters available for definition in the command line for each camera are described in rs_launch.configurable_parameters
# For each device, the parameter name was changed to include an index.
# For example: to set camera_name for device1 set parameter camera_name1.
# command line example:
# ros2 launch realsense2_camera rs_multi_camera_launch.py camera_name1:=D400 device_type2:=l5. device_type1:=d4..

"""Launch realsense2_camera node."""
import copy
from launch import LaunchDescription, LaunchContext
import launch_ros.actions
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import os
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))
import rs_launch

local_parameters = [{'name': 'camera_name1',        'default': 'cam_left',          'description': 'camera1 unique name'},
                    {'name': 'camera_name2',        'default': 'cam_mid',           'description': 'camera2 unique name'},
                    {'name': 'camera_name3',        'default': 'cam_right',         'description': 'camera3 unique name'},
                    {'name': 'camera_namespace1',   'default': 'realsense1',        'description': 'camera1 namespace'},
                    {'name': 'camera_namespace2',   'default': 'realsense2',        'description': 'camera2 namespace'},
                    {'name': 'camera_namespace3',   'default': 'realsense3',        'description': 'camera3 namespace'},
                    {'name': 'serial_no1',          'default': '_215222077504',     'description': 'camera1 serial number'},
                    {'name': 'serial_no2',          'default': '_949122070619',     'description': 'camera2 serial number'},
                    {'name': 'serial_no3',          'default': '_213622074227',     'description': 'camera3 serial number'},
                    {'name': 'config_file1',        'default': '/home/realsense/vision-ws/src/realsense-ros/realsense2_camera/launch/config/multi_cam_config.yaml', 'description': 'camera1 config file'},
                    {'name': 'config_file2',        'default': '/home/realsense/vision-ws/src/realsense-ros/realsense2_camera/launch/config/multi_cam_config.yaml', 'description': 'camera2 config file'},
                    {'name': 'config_file3',        'default': '/home/realsense/vision-ws/src/realsense-ros/realsense2_camera/launch/config/multi_cam_config.yaml', 'description': 'camera3 config file'},
                    # tf parameters for left & right cameras
                    {'name': 'tf.translation1.x',    'default': '0.08',             'description': 'x'},
                    {'name': 'tf.translation1.y',    'default': '0.0',              'description': 'y'},
                    {'name': 'tf.translation1.z',    'default': '0.02966025404',    'description': 'z'},
                    {'name': 'tf.rotation1.yaw',     'default': '0.0',              'description': 'yaw'},
                    {'name': 'tf.rotation1.pitch',   'default': '-0.5235987756',    'description': 'pitch'},
                    {'name': 'tf.rotation1.roll',    'default': '0.0',              'description': 'roll'},
                    {'name': 'tf.translation3.x',    'default': '0.08',             'description': 'x'},
                    {'name': 'tf.translation3.y',    'default': '0.0',              'description': 'y'},
                    {'name': 'tf.translation3.z',    'default': '-0.02966025404',   'description': 'z'},
                    {'name': 'tf.rotation3.yaw',     'default': '0.0',              'description': 'yaw'},
                    {'name': 'tf.rotation3.pitch',   'default': '0.5235987756',     'description': 'pitch'},
                    {'name': 'tf.rotation3.roll',    'default': '0.0',              'description': 'roll'},
                    ]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

def launch_static_transform_publisher_node(context: LaunchContext, param_name_suffix=''):
    # Create static transform publisher node with adjusted camera names
    node = launch_ros.actions.Node(
        name=context.launch_configurations['camera_name' + param_name_suffix] + '_transform_publisher',
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            context.launch_configurations['tf.translation'+ param_name_suffix +'.x'],
            context.launch_configurations['tf.translation'+ param_name_suffix +'.y'],
            context.launch_configurations['tf.translation'+ param_name_suffix +'.z'],
            context.launch_configurations['tf.rotation'+ param_name_suffix +'.yaw'],
            context.launch_configurations['tf.rotation'+ param_name_suffix +'.pitch'],
            context.launch_configurations['tf.rotation'+ param_name_suffix +'.roll'],
            context.launch_configurations['camera_name2'] + "_link",
            context.launch_configurations['camera_name' + param_name_suffix] + '_link'
        ]
    )
    return [node]

def launch_camera_base_transform_publisher_node(context: LaunchContext):
    node = launch_ros.actions.Node(
        name='camera_base_transform_publisher',
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            '0', '0.048', '1.44', '1.5707963268', '1.0471975512', '-1.5707963268',
            'camera_base_link',
            context.launch_configurations['camera_name2'] + '_link'
        ]
    )
    return [node]

def launch_map_transform_publisher_node(context: LaunchContext):
    node = launch_ros.actions.Node(
        name='map_transform_publisher',
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            '1.725', '2.127', '0', '0', '0', '1', '0',
            'map',
            'camera_base_link'
        ]
    )
    return [node]

def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    params3 = duplicate_params(rs_launch.configurable_parameters, '3')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) +
        rs_launch.declare_configurable_parameters(params3) +
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params1),
                                 'param_name_suffix': '1'}),
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params2),
                                 'param_name_suffix': '2'}),
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params3),
                                 'param_name_suffix': '3'}),
        OpaqueFunction(function=launch_static_transform_publisher_node,
                       kwargs = {'param_name_suffix': '1'}),
        OpaqueFunction(function=launch_static_transform_publisher_node,
                       kwargs = {'param_name_suffix': '3'}),     
        OpaqueFunction(function=launch_camera_base_transform_publisher_node),   
        OpaqueFunction(function=launch_map_transform_publisher_node)              
    ])