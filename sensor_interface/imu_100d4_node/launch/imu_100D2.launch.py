from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_node',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[
                {'port': '/dev/ttyUSB2'},
                {'model': '100D4'},
                {'baud': 115200},
                {'gravity': 9.81},
                {'use_magnetic': 0}
            ]
        )
    ])

