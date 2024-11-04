首先，我们先下载cartographer的cartographer包和cartographer_ros包

打开终端

```
cd ros2_ws/src
mkdir cartographer_core
git clone https://github.com/cartographer-project/cartographer.git
git clone https://github.com/cartographer-project/cartographer_ros.git
```

可能由于网络问题，clone失败，可以多尝试几次即可成功

接下来，我们创建一个库：

```
ros2 pkg create mick_cartographer --build-type ament_cmake --dependencies rclcpp
cd mick_cartographer
mkdir launch map config rviz
```

然后是启动文件的创建：

```
cd launch 
touch cartographer.launch.py
```

完整的cartographer.launch.py启动文件：

```
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    #根据自己的包名进行修改
    pkg_share = FindPackageShare(package='mick_cartographer').find('mick_cartographer')
     
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    configuration_basename = LaunchConfiguration('configuration_basename', default='mick_cartographer.lua')

    #声明节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
        
        
    start_robot_state_publisher_laser_to_base =  Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
        )
    start_robot_state_publisher_odom_to_map =  Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
        )
    start_robot_state_publisher_base_to_odom =  Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
        )
                
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/home/meteor/ros2_ws/src/mick_cartographer/rviz/config.rviz'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')
                     
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)
    ld.add_action(start_robot_state_publisher_laser_to_base)
    ld.add_action(start_robot_state_publisher_odom_to_map)
    ld.add_action(start_robot_state_publisher_base_to_odom)
    return ld
```

创建并编写配置文件：

```
cd ..
cd config
touch mick_cartographer.lua
```

.lua文件是cartographer中用于参数配置的文件

以下是对一些参数的介绍说明：

|      |              基本参数               |                             说明                             |
| :--: | :---------------------------------: | :----------------------------------------------------------: |
|  1   |            **map_frame**            | 用来发布子图，是poses的父帧，通常是“map”。和odom最开始的时候是一个原点，但时间累计对产生累积误差。 |
|  2   |         **tracking_frame**          | 由SLAM算法跟踪的坐标系。如果使用IMU，尽管它可能是旋转的，它也应该在其位置。一个常见的选择是“imu_link”。 |
|  3   |         **published_frame**         | 这个frame是用来正在发布poses的子帧，和map_frame对应。一般就将其设置为"base_link",就是"map->base_link"，或者"map->odom->base_link" |
|  4   |           **odom_frame**            | 仅在provide_odom_frame为true时使用。published_frame 和 map_frame之间的框架，用于发布(非循环关闭)local SLAM结果。默认是“odom”。 |
|  5   |       **provide_odom_frame**        | 如果启用，则local-slam估计的连续的姿态(不包括回环)将作为map_frame中 odom_frame发布。 |
|  6   |  **publish_frame_projected_to_2d**  | 如果启用，则已发布的pose将限制为纯2D姿势(无滚动，俯仰或z偏移)。这可以防止在2D模式中由于"pose extrapolation step"而发生的"out-of-plane"(如果姿势应该作为类似“base-footprint”发布)。 |
|  7   |          **use_odommetry**          | 如果启用，请在主题“odom”上订阅nav_msgs/Odometry。在这种情况下必须提供"Odommetry"，并且信息将包含在SLAM中。 |
|  8   |           **use_nav_sat**           | 如果启用，请在主题“fix”上订阅sensor_msgs / NavSatFix。 在这种情况下必须提供Navigation data，并且信息将包含在全局SLAM中。 |
|  9   |          **use_landmarks**          | 如果启用，请在主题“Landmarks”上订阅cartographer_ros_msgs / LandmarkList。在这种情况下必须提供Landmarks，并且信息将包含在SLAM中。 |
|  10  |         **num_laser_scans**         | 订阅的激光扫描主题数量。在一个激光扫描仪的“扫描”主题上订阅sensor_msgs /LaserScan或在多个激光扫描仪上订阅主题“scan_1”，“scan_2”等。 |
|  11  | **num_subdivisions_per_laser_scan** | 将每个接收到的(multi-echo)激光扫描分割成的点云数。细分扫描可以在扫描仪移动时取消扫描获取的扫描。有一个相应的轨迹构建器选项可将细分扫描累积到"用于scan_matching的点云"中。 |
|  12  |   **num_multi_echo_laser_scans**    | 订阅的multi-echo激光扫描主题的数量。在一个激光扫描仪的“echoes”主题上订阅sensor_msgs / MultiEchoLaserScan，或者为多个激光扫描仪订阅主题“echoes_1”，“echoes_2”等。 |

|      |         时间间隔设置参数          |                             说明                             |
| :--: | :-------------------------------: | :----------------------------------------------------------: |
|  1   | **lookup_transform_timeout_sec**  |                  使用tf2查找变换的超时时间                   |
|  2   |   **submap_publish_period_sec**   |                 发布子图的时间间隔，单位是秒                 |
|  3   |    **pose_publish_period_sec**    |          发布pose的时间间隔，比如：5e-3频率是200Hz           |
|  4   | **trajectory_publish_period_sec** | 以秒为单位发布轨迹标记的间隔，例如， 30e-3<br />其中3.30e-3是科学记数法，表示的是30乘以10的-3次方。也就是0.030秒，就是30毫秒。 |

|      |        采样比率相关参数        |                             说明                             |
| :--: | :----------------------------: | :----------------------------------------------------------: |
|  1   | **rangefinder_sampling_ratio** | Fixed ratio sampling for range finders messages（测距仪消息的固定比率采样） |
|  2   |  **odometry_sampling_ratio**   | Fixed ratio sampling for odometry messages（里程计消息的固定比率采样） |
|  3   | **fixed_frame_sampling_ratio** | Fixed ratio sampling for fixed frame messages（固定帧消息的固定比率采样） |
|  4   |     **imu_sampling_ratio**     | Fixed ratio sampling for IMU messages（IMU 消息的固定比率采样） |
|  5   |  **landmarks_sampling_ratio**  | Fixed ratio sampling for landmarks messages（地标消息的固定比率采样） |

完整的mick_cartographer.lua配置文件：

```

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  -- true改为false，不用提供里程计数据
  provide_odom_frame = false,
  -- false改为true，仅发布2D位资
  publish_frame_projected_to_2d = true,
  -- false改为true，使用里程计数据
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  -- 0改为1,使用一个雷达
  num_laser_scans = 1,
  -- 1改为0，不使用多波雷达
  num_multi_echo_laser_scans = 0,
  -- 10改为1，1/1=1等于不分割
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


-- false改为true，启动2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

-- 0改成0.10,比机器人半径小的都忽略
TRAJECTORY_BUILDER_2D.min_range = 0.10
-- 30改成3.5,限制在雷达最大扫描范围内，越小一般越精确些
TRAJECTORY_BUILDER_2D.max_range = 3.5
-- 5改成3,传感器数据超出有效范围最大值
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- true改成false,不使用IMU数据，大家可以开启，然后对比下效果
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- false改成true,使用实时回环检测来进行前端的扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- 1.0改成0.1,提高对运动的敏感度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- 0.55改成0.65,Fast csm的最低分数，高于此分数才进行优化。
POSE_GRAPH.constraint_builder.min_score = 0.65
--0.6改成0.7,全局定位最小分数，低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 设置0可关闭全局SLAM
-- POSE_GRAPH.optimize_every_n_nodes = 0

return options
```

对于CmakeLists文件的修改

需要添加：

```
install(
  DIRECTORY config launch rviz map
  DESTINATION share/${PROJECT_NAME}
)
```

对于package文件的修改

需要添加：

```
<exec_depend>cartographer_ros</exec_depend>
```

这样，cartographer包就配置好了

下面编译工作空间

```
cd ros2_ws
colcon build
source install/setup.bash
```

同样的我们先启动小车底盘节点和雷达的启动文件

```
ros2 run mick_bringup mick_bringup
```

```
ros2 launch sllidar_ros2 sllidar_a2m7_launch.py
```

接着我们启动cartographer建图启动文件

```
ros2 launch mick_cartographer cartographer.launch.py
```

<img src="README.assets\rviz2_crato.png" style="zoom:67%;" />

这里rviz的topic和type可以依照gmapping一样进行配置，或者可以直接保存其配置文件并使用

这是rviz配置文件的示例：

```
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Status1
        - /Odometry1
        - /Map1
      Splitter Ratio: 0.5
    Tree Height: 585
  - Class: rviz_common/Selection
    Name: Selection
  - Class: rviz_common/Tool Properties
    Expanded:
      - /2D Goal Pose1
      - /Publish Point1
    Name: Tool Properties
    Splitter Ratio: 0.5886790156364441
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Angle Tolerance: 0.10000000149011612
      Class: rviz_default_plugins/Odometry
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 255; 127
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.30000001192092896
          Color: 204; 51; 204
          Scale: 1
          Value: true
        Value: true
      Enabled: true
      Keep: 10000
      Name: Odometry
      Position Tolerance: 0.10000000149011612
      Shape:
        Alpha: 1
        Axes Length: 1
        Axes Radius: 0.10000000149011612
        Color: 255; 25; 0
        Head Length: 0.30000001192092896
        Head Radius: 0.10000000149011612
        Shaft Length: 1
        Shaft Radius: 0.05000000074505806
        Value: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /odom
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 255; 255
      Color Transformer: Intensity
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 47
      Min Color: 0; 0; 0
      Min Intensity: 47
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.009999999776482582
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    - Alpha: 0.699999988079071
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
      Value: true
    - Class: rviz_default_plugins/Axes
      Enabled: true
      Length: 1
      Name: base_link_Axes
      Radius: 0.10000000149011612
      Reference Frame: base_link
      Value: true
    - Class: rviz_default_plugins/Axes
      Enabled: false
      Length: 0.5
      Name: laser_Axes
      Radius: 0.05000000074505806
      Reference Frame: laser
      Value: false
    - Class: rviz_default_plugins/Axes
      Enabled: false
      Length: 0.5
      Name: map_Axes
      Radius: 0.05000000074505806
      Reference Frame: map
      Value: false
    - Class: rviz_default_plugins/Axes
      Enabled: true
      Length: 2
      Name: odom_Axes
      Radius: 0.07000000029802322
      Reference Frame: odom
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
    - Class: rviz_default_plugins/PublishPoint
      Single click: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /clicked_point
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 18.357418060302734
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 0.6353980898857117
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 1.7403970956802368
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 814
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002d4fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002d4000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002d4fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002d4000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000004c5000002d400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1846
  X: 72
  Y: 27
```



下面我们就可以移动小车开始实际建图了

<img src="README.assets\carto1.gif" style="zoom:150%;" />



建好图后再使用map_server进行保存

```
mkdir map
cd map
mkdir map_learn 
cd map_learn
ros2 run nav2_map_server map_saver_cli
```



以上就是完整的cratographer的实际建图