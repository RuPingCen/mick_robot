下面是利用实体小车底盘发布的odom进行gmapping建图：

首先，连接小车和电脑并启动小车节点（依旧需要对串口进行赋权限）

然后我们可以借助gtkterm串口工具，检测是否收到小车上传的数据

```
sudo apt install gtkterm
gtkterm
```

这样我们就可以看到如下的可视化界面

<img src="README.assets\gtkterm.png" style="zoom: 67%;" />

接着

<img src="README.assets\gtk1.png" style="zoom:67%;" />

在打开的选项卡中选择对应的串口：

<img src="README.assets\gtk2.png" style="zoom:67%;" />

<img src="README.assets\gtk3.gif" style="zoom: 80%;" />

这样，就表示能够成功的收到小车底盘上传的数据。

然后我们启动雷达和gmapping的启动文件，

```
source install/setup.bash
ros2 launch sllidar_ros2 sllidar_a2m7_launch.py
```

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





