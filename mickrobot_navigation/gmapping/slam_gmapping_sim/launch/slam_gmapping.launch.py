from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import launch_ros.actions

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    rviz_config_file = PathJoinSubstitution(
           [FindPackageShare("slam_gmapping_sim"), "rviz", "gmapping.rviz"]
   )
   
    gmapping_config_file = PathJoinSubstitution(
           [FindPackageShare("slam_gmapping_sim"), "config", "gmapping_params.yaml"]
   )
    return LaunchDescription([
        Node(
            package='slam_gmapping_sim',
            executable='slam_gmapping_sim',
            name='slam_gmapping',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('/scan', '/scan'),
            ],
            arguments=[
                '__params:=',gmapping_config_file
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        # ),
        #  Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        #     output='screen'
        # ),

        #  Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_node'],
        #     output='screen'
        # ),
    ])

