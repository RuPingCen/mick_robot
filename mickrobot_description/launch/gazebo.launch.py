import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    robot_name_in_model = 'mickrobot'
    package_name = 'mickrobot_description'
    urdf_gazebo_name = "mickrobot_stl.urdf"
    urdf_rviz_name = "mickrobot_stl_rviz.urdf"

    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_gazebo_name}')
    urdf_model_rviz_path = os.path.join(pkg_share, f'urdf/{urdf_rviz_name}')
    world_path = os.path.join(pkg_share, 'worlds', 'nav.world') 

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_gazebo.rviz')

    # Gazebo Server
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
   # Spawn robot into Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', robot_name_in_model, '-file', urdf_model_path],
        #arguments=['-entity', robot_name_in_model, '-topic', '/robot_description'],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    # A GUI to manipulate the joint state values
    # start_joint_state_publisher_gui_node = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui')
    
     # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True,'robot_description': Command(['xacro ', urdf_model_rviz_path])}],
        arguments=[urdf_model_rviz_path]
    )
  
    # Launch RViz
    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
        )
    ld.add_action(start_gazebo_cmd)
    ld.add_action(joint_state_publisher_node)
    #ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_rviz_cmd)
    
    return ld
