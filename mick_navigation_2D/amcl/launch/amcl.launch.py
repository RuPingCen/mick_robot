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

 
def generate_launch_description():
 
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('amcl'), 'rviz', 'amcl.rviz']
    )

    default_map_path = PathJoinSubstitution(
        [FindPackageShare('amcl'), 'maps', 'test_map.yaml']
    )

    amcl_param_path = PathJoinSubstitution(
        [FindPackageShare('amcl'), 'param', 'amcl.yaml']
    )

    return LaunchDescription([
 
        # map_server 节点
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            arguments=['--params-file ', rviz_config_path]
            #parameters=[default_map_path]
        ),
   
        # # amcl 节点
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_param_path]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'node_names': ['map_server','amcl']}])

    ])
