首先，我们先下载cartographer的cartographer包和cartographer_ros包

安装gtest

```
wget https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip

unzip googletest-1.14.0.zip
cd googletest-1.14.0
mkdir build && cd build
cmake ..
make
sudo make install
```
安装 abseil-cpp 
git clone https://github.com/abseil/abseil-cpp.git
cd abseil-cpp
mkdir build
cd build
cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON
make 
sudo make install

安装ceres 

```
sudo apt-get install  liblapack-dev libsuitesparse-dev libcxsparse3 libgflags-dev libgoogle-glog-dev libgtest-dev

cd ~
git clone https://github.com/ceres-solver/ceres-solver
git checkout 399cda7
cd ceres-solver && mkdir build
cd build
cmake ..
make
sudo make install
```

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


下面我们就可以移动小车开始实际建图了

<img src="README.assets\carto1.gif" style="zoom:150%;" />



建好图后再使用map_server进行保存

```
mkdir map
cd map
ros2 run nav2_map_server map_saver_cli
```

成功运行以后会在目录下生成一个 “xx.png” 和 “xx.yaml” 的文件。


以上就是完整的cratographer的实际建图