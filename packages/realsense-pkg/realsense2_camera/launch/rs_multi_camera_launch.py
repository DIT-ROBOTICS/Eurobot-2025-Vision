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
import rs_launch

local_parameters = [{'name': 'camera_name1',        'default': '',          'description': 'camera1 unique name'},
                    {'name': 'camera_name2',        'default': '',          'description': 'camera2 unique name'},
                    {'name': 'camera_namespace1',   'default': '',      'description': 'camera1 namespace'},
                    {'name': 'camera_namespace2',   'default': '',       'description': 'camera2 namespace'},
                    {'name': 'serial_no1',          'default': '', 'description': 'camera1 serial number'},
                    {'name': 'serial_no2',          'default': '', 'description': 'camera2 serial number'},
                    {'name': 'enable_color1',           'default': 'true', 'description': 'enable color stream'},
                    {'name': 'enable_color2',           'default': 'true', 'description': 'enable color stream'},
                    {'name': 'enable_depth1',           'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'enable_depth2',           'default': 'true', 'description': 'enable depth stream'},
                    {'name': 'pointcloud.enable1',      'default': 'true', 'description': 'enable pointcloud'},
                    {'name': 'pointcloud.enable2',      'default': 'true', 'description': 'enable pointcloud'},
                    {'name': 'decimation_filter.enable1',     'default': 'true', 'description': 'enable_decimation_filter'},
                    {'name': 'decimation_filter.enable2',     'default': 'true', 'description': 'enable_decimation_filter'},
                    {'name': 'spatial_filter.enable1',  'default': 'true', 'description': 'enable_spatial_filter'},
                    {'name': 'spatial_filter.enable2',  'default': 'true', 'description': 'enable_spatial_filter'},
                    {'name': 'temporal_filter.enable1', 'default': 'true', 'description': 'enable_temporal_filter'},
                    {'name': 'temporal_filter.enable2', 'default': 'true', 'description': 'enable_temporal_filter'},
                    {'name': 'rgb_camera.color_profile1',     'default': '848,480,60', 'description': 'color stream profile'},
                    {'name': 'rgb_camera.color_format1',      'default': 'RGB8', 'description': 'color stream format'},
                    {'name': 'rgb_camera.enable_auto_exposure1', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                    {'name': 'rgb_camera.color_profile2',     'default': '848,480,60', 'description': 'color stream profile'},
                    {'name': 'rgb_camera.color_format2',      'default': 'RGB8', 'description': 'color stream format'},
                    {'name': 'rgb_camera.enable_auto_exposure2', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                    {'name': 'depth_module.depth_profile1',   'default': '848,480,60', 'description': 'depth stream profile'},
                    {'name': 'depth_module.depth_format1',    'default': 'Z16', 'description': 'depth stream format'},
                    {'name': 'depth_module.depth_profile2',   'default': '848,480,60', 'description': 'depth stream profile'},
                    {'name': 'depth_module.depth_format2',    'default': 'Z16', 'description': 'depth stream format'},
                    {'name': 'depth_module.enable_auto_exposure1', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                    {'name': 'depth_module.enable_auto_exposure2', 'default': 'true', 'description': 'enable/disable auto exposure for depth image'},
                    {'name': 'align_depth.enable1',           'default': 'true', 'description': 'enable align depth filter'},
                    {'name': 'align_depth.enable2',           'default': 'true', 'description': 'enable align depth filter'},
                    ]

def set_configurable_parameters(local_params):
    return dict([(param['original_name'], LaunchConfiguration(param['name'])) for param in local_params])

def duplicate_params(general_params, posix):
    local_params = copy.deepcopy(general_params)
    for param in local_params:
        param['original_name'] = param['name']
        param['name'] += posix
    return local_params

def launch_static_transform_publisher_node(context : LaunchContext):
    # dummy static transformation from camera1 to camera2
    node = launch_ros.actions.Node(
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0",
                          context.launch_configurations['camera_name1'] + "_link",
                          context.launch_configurations['camera_name2'] + "_link"]
    )
    return [node]

def generate_launch_description():
    params1 = duplicate_params(rs_launch.configurable_parameters, '1')
    params2 = duplicate_params(rs_launch.configurable_parameters, '2')
    return LaunchDescription(
        rs_launch.declare_configurable_parameters(local_parameters) +
        rs_launch.declare_configurable_parameters(params1) +
        rs_launch.declare_configurable_parameters(params2) +
        [
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params1),
                                 'param_name_suffix': '1'}),
        OpaqueFunction(function=rs_launch.launch_setup,
                       kwargs = {'params'           : set_configurable_parameters(params2),
                                 'param_name_suffix': '2'}),
        OpaqueFunction(function=launch_static_transform_publisher_node)
    ])
