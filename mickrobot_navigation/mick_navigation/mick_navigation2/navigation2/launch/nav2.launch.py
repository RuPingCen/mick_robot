# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


MAP_NAME='rikibot' #change to the name of your own map here

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])
        ]),

    )


    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('racecar'), 'rviz', 'navigation.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('racecar'), 'map', f'{MAP_NAME}.yaml']
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('racecar'), 'config/nav', 'navigation.yaml']
    )

    nav2_sim_config_path = PathJoinSubstitution(
        [FindPackageShare('racecar'), 'config/nav', 'navigation_sim.yaml']
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='false',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='false',
            description='Run rviz'
        ),

       DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=UnlessCondition(LaunchConfiguration("sim")),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=IfCondition(LaunchConfiguration("sim")),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_sim_config_path
            }.items()
        ),

   	    Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',output='screen',
            arguments = ['0', '0', '0', '0', '0', '0', 'odom', 'map']
        ),

        Node(
            package = 'tf2_ros',
            namespace = 'scan_to_map',
            executable = 'static_transform_publisher',output='screen',
            arguments = ['0', '0', '0', '0', '0', '0', 'map', 'scan']
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_launch_file_dir, '/lidar_laser.launch.py']),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        )
    ])
