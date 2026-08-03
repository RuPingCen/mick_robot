"""Launch file for mickrobot_bringup Python node.
Mirrors mickrobotx4_ros2.launch.py but targets the Python executable.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mickrobot_bringup',
            executable='mickrobot_bringup_py',
            name='mickrobot_bringup',
            output='screen',
            parameters=[
                {'dev': '/dev/ttyUSB0'},
                {'baud': 115200},
                {'chassis_type': 0},
                {'is_pub_path': 0},
                {'communication_mode': 1},
                {'sub_cmdvel_topic': 'mickrobot/cmd_vel'},
                {'pub_odom_topic': 'mickrobot/odom'},
                {'pub_imu_topic': 'mickrobot/Imu'},
                {'joy_topic': 'mickrobot/rc_remotes/joy'},
                {'pub_twist_topic': 'mickrobot/odom/twist'},
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2base_link',
            arguments=['0.0', '0.0', '0.15', '0.0', '0.0', '0.0',
                       'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2laser_link',
            arguments=['0.07', '0.0', '0.0', '0.0', '0.0', '0.0',
                       'base_link', 'laser']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2imu',
            arguments=['0.1653', '0.0', '0.0', '0.0', '0.0', '0.0',
                       'base_link', 'imu_node']
        ),
    ])
