# /home/mickrobot/ros2_mick_ws/src/mick_robot/apps/path_pub/launch/path_pub.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """启动path_pub节点，支持路径和频率参数"""
    
    # 声明launch参数
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('path_pub'),
            'parameters',  # parameters 子目录
            'octagon_path.txt'     # 文件名
        ]),
        description='Path to the trajectory file'
    )
    
    publish_freq_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='1.0',
        description='Frequency at which the path is published (Hz)'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/reference_path',
        description='Topic name to publish the path'
    )

    # 获取launch配置
    path_file = LaunchConfiguration('path_file')
    publish_frequency = LaunchConfiguration('publish_frequency')
    topic_name = LaunchConfiguration('topic_name')

    return LaunchDescription([
        path_file_arg,
        publish_freq_arg,
        topic_name_arg,
        Node(
            package='path_pub',
            executable='path_pub',
            name='path_pub_node',
            parameters=[
                {'path_file_path': path_file},
                {'publish_frequency': publish_frequency},
                {'topic_name': topic_name}
            ],
            output='screen'
        ),
    ])
