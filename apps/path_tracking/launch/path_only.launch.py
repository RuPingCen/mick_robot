from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """仅启动路径发布器"""
    
    return LaunchDescription([
        Node(
            package='path_tracking',
            executable='circle_path_publisher',
            name='circle_path_publisher',
            output='screen'
        ),
    ])
