from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 获取oradar_lidar包的共享目录，以便访问其launch文件和资源
    oradar_lidar_share = get_package_share_directory('oradar_lidar')

    # 包含ms200_scan.launch文件
    ms200_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([oradar_lidar_share, '/launch/ms200_scan.launch.py']))

    # 包含rf2o_laser_odometry.launch文件
    rf2o_laser_odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('rf2o_laser_odometry'), '/launch/rf2o_laser_odometry.launch.py']))

    # 使用static_transform_publisher发布base_link到laser的静态TF变换
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "1", "0", "0", "0", "/base_link", "/laser"]
    )

    # 包含gmapping.launch文件
    gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([oradar_lidar_share, '/launch/gmapping.launch.py']))

    # 启动RViz，并加载配置文件
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", [oradar_lidar_share, "/rviz/test_map.rviz"]]
    )

    # 创建LaunchDescription对象，包含所有要启动的节点和动作
    ld = LaunchDescription([
        ms200_scan_launch,
        static_transform_publisher,
        gmapping_launch,
        rviz
    ])

    return ld
