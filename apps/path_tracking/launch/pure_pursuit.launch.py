from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """启动路径发布器和Pure Pursuit控制器"""
    
    return LaunchDescription([
        # 圆形路径发布器
        Node(
            package='path_tracking',
            executable='circle_path_publisher',
            name='circle_path_publisher',
            output='screen'
        ),
        
        # Pure Pursuit控制器
        Node(
            package='path_tracking',
            executable='pure_pursuit_controller',
            name='pure_pursuit_controller',
            output='screen',
            parameters=[{
                'lookahead_distance': 0.6,
                'linear_velocity': 0.6,
                'max_angular_velocity': 3.5,
                'min_linear_velocity': 0.45,
                'position_tolerance': 0.12,
                'yaw_tolerance': 0.05
            }]
        ),
    ])
