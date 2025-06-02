gmapping 建图需要提供2D激光雷达 和 里程计数据。2D 激光点云数据来源于激光雷达节点,里程计数据由小车底盘发布。

里程计数据类型：nav_msgs/msg/Odometry

激光点云数据类型：sensor_msgs/msg/LaserScan



下面是利用实体小车底盘发布的odom进行gmapping建图步骤：

# 1. 检查小车连接

首先，连接小车和电脑并启动小车节点（依旧需要对串口进行赋权限）

```
sudo chmod 777 /dev/ttyUSB*
ros2 launch mickrobot_bringup mickrobotx4.launch.py
```

成功启动以后，终端会实时打印里程计信息

<img src="README.assets\chasiss1.png" style="zoom: 80%;" />



这样，就表示能够成功的收到小车底盘上传的数据。

# 2. 启动雷达
## 2.1 思岚A2M7激光雷达
然后我们启动雷达和gmapping的启动文件，

```
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_a2m7_launch.py
```

通常激光雷达成功启动以后会一直旋转。

<img src="README.assets\lidar-A2M7.png" style="zoom: 80%;" />

## 2.2 MS200激光雷达

注意将激光雷达的方向要与小车的X方向保持一致

```
ros2 launch oradar_lidar ms200_scan_view.launch.py  
```

<img src="README.assets\lidar-MS200.png" style="zoom: 80%;" />

# 3. 启动gmapping建图节点

打开另一终端

```
source install/setup.bash
ros2 launch slam_gmapping slam_gmapping.launch.py
```

接着我们依旧添加一些topic和type：

<img src="README.assets\gmap.png" style="zoom: 100%;" />

在map的topic中选中/map话题，接着我们就可以看见成功的建图了：

<img src="README.assets\gamp1.png" style="zoom: 50%;" />

我们接下来就可以移动小车对实际的物理环境进行建图了

<img src="README.assets\gmap2.gif" style="zoom: 100%;" />

# 4. 保存地图

建好图后再使用map_server进行保存

首先安装navigation2库

```
sudo apt info ros-$ROS_DISTRO-navigation2
sudo apt info ros-$ROS_DISTRO-nav2-bringup
```

$ROS_DISTRO替换为自己的ros2版本

```
mkdir map
cd map
mkdir map_learn 
cd map_learn
ros2 run nav2_map_server map_saver_cli
```

如果出现报错，我们可以下载navigation2中的nav2_map_server的源码

并在map_saver.cpp文件中加入一行代码

```
auto mapCallback = [&prom](
      const nav_msgs::msg::OccupancyGrid::SharedPtr msg) -> void {
        prom.set_value(msg);
      };

    rclcpp::QoS map_qos(10);  // initialize to default
    if (map_subscribe_transient_local_) {
      map_qos.transient_local();
      map_qos.reliable();
      map_qos.keep_last(1); 
      map_qos.durability_volatile();  //加入这一行代码
    }
```

保存并编译就可以正常运行map_server了

重新运行

```
ros2 run nav2_map_server map_saver_cli
```

<img src="README.assets\map_server.png" style="zoom: 100%;" />

<img src="README.assets\map_server1.png" style="zoom: 100%;" />

其中有.pgm和.yaml文件，后续导航需要用到.yaml文件

到这，gmapping建图就完成了

需要注意的是，建图过程中需要避免玻璃和电梯门等穿透性、高反射性地物体，它们会影响建图的质量

<img src="README.assets\map_server2.png" style="zoom: 80%;" />

这是一个建好并保存好的地图示例





