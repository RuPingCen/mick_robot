from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """启动路径发布器和Stanley控制器"""
    
    return LaunchDescription([
    
        
        # Stanley控制器
        Node(
            package='path_tracking',
            executable='stanley_controller',
            name='stanley_controller',
            output='screen',
            parameters=[{
                'k_e': 1.5,
                'k_v': 0.1,
                'linear_velocity': 0.6,
                'max_angular_velocity': 3.0,
                'wheelbase': 0.52,
                'lookahead_gain': 0.3
            }]
        ),
    ])
