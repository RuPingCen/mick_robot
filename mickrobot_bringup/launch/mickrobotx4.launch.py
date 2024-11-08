from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mickrobot_bringup',
            executable='mickrobot_bringup',
            name='mickrobot_bringup',
            output='screen',
            parameters=[
                {'dev': '/dev/ttyUSB0'},
                {'baud': 115200},
                {'chassis_type': 0},
                {'is_pub_path': 0},
                {'sub_cmdvel_topic': 'mickrobot/cmd_vel'},
                {'pub_odom_topic': 'mickrobot/odom'},
                {'pub_imu_topic': 'mickrobot/Imu'},
                {'joy_topic': 'mickrobot/rc_remotes/joy'} 
            ]
        )
    ])

 
