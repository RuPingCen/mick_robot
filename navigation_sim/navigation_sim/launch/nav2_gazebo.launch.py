import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution  # 确保导入PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare  

def generate_launch_description():
    navigation_pkg = 'navigation_sim'
    racecar_dir = get_package_share_directory('navigation_sim')
    racecar_launchr = os.path.join(racecar_dir, 'launch')
    # 统一使用Substitution处理路径
    rviz_config = PathJoinSubstitution([
        FindPackageShare(navigation_pkg),
        'rviz',
        'navigation2.rviz'  
    ])
    
    param_file = PathJoinSubstitution([
        FindPackageShare(navigation_pkg),
        'param',
        'nav_gazebo.yaml'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [racecar_launchr, '/bringup_launch.py']),
            launch_arguments={
                'map': PathJoinSubstitution([
                    FindPackageShare(navigation_pkg),
                    'maps',
                    'map_out.yaml'
                ]),
                'use_sim_time': 'True',
                'params_file': param_file}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}]  # 显式声明仿真时间[7](@ref)
        )
    ])
