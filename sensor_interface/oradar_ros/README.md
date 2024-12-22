# ORADAR ROS package

ORADAR ROS 包用于连接Oradar MS200激光雷达，此ROS包支持ROS和ROS2环境。其中ROS支持 Indigo，Kinetic，Melodic等ROS版本；ROS2支持Ubuntu 20.04 ROS2 foxy版本及以上。

## 使用方法： 

1. 在系统中安装ROS环境，具体安装方法参考下面连接：

   ROS安装链接：http://wiki.ros.org/kinetic/Installation/Ubuntu 

   ROS2安装链接：https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

   **请不要在一台电脑上同时安装ROS和ROS2，以避免可能的版本冲突和手工安装其他库的麻烦**

2. 将oradar_ros源码复制到ros工作目录下的src目录，修改相应文件

   ```shell
   mkdir -p ~/lidar_ros_ws/src
   cp -ar oradar_ros ~/lidar_ros_ws/src/
   ```

   (1) 当使用ROS时，需要打开oradar_ros源码根目录下的*CMakeLists.txt*文件，将文件顶部的变量**COMPILE_METHOD**改为**CATKIN**，

   ```cmake
   #=======================================
   # Compile setup (ORIGINAL,CATKIN,COLCON)
   #=======================================
   set(COMPILE_METHOD CATKIN)
   ```

   然后把*package_ros1.xml*文件复制一份，命名为为*package.xml*。

   (2) 当使用ROS2时，需要打开oradar_ros源码根目录下的*CMakeLists.txt*文件，将文件顶部的变量**COMPILE_METHOD**改为**COLCON**，

   ```cmake
   #=======================================
   # Compile setup (ORIGINAL,CATKIN,COLCON)
   #=======================================
   set(COMPILE_METHOD COLCON)
   ```

   然后把*package_ros2.xml*文件复制一份，命名为*package.xml*。


3. 编译工程、设置环境变量

   当环境是ROS时：

   ```shell
   cd ~/lidar_ros_ws
   catkin_make
   source devel/setup.sh
   ```

   当环境是ROS2时：

   ```
   cd ~/lidar_ros_ws
   colcon build
   source install/setup.bash
   ```

4. 配置上位机串口

   配置串口port_name和波特率： 默认配置port_name为/dev/ttyACM0, 波特率为230400
   将ms200激光雷达设备通过USB转串口线，插入连接到Ubuntu系统，在Ubuntu系统下打开终端，输入 `ls /dev/ttyACM*` 查看串口设备是否接入，若检测到串口设备，则使用 `sudo chmod 777 /dev/ttyACM*` 命令赋予最高权限。

5. 配置雷达参数

   打开oradar_ros/launch/ms200_scan.launch 进行参数配置或者oradar_ros/launch/ms200_scan.launch.py 进行参数配置

   参数说明如下：

   | 参数名      | 数据类型 | 描述                                                         |
   | ----------- | -------- | ------------------------------------------------------------ |
   | frame_id    | string   | 激光雷达坐标系名称。 默认为laser_frame                       |
   | scan_topic  | string   | LaserScan主题名。 默认为scan                                 |
   | port_name   | string   | 激光雷达串口名称。 默认值为/dev/ttyACM0                      |
   | baudrate    | int      | 雷达串口波特率.。 默认值为230400                             |
   | angle_min   | double   | 最小角度，单位度，取值范围[0, 360]。 默认值为0 |
   | angle_max   | double   | 最大角度，单位度，取值范围[0, 360]。 默认值为360 |
   | range_min   | double   | 最小距离，单位米，默认值为0.05                               |
   | range_max   | double   | 最大距离，单位米，默认值为20.0                               |
   | clockwise    | bool     | 配置点云方向，true为顺时针， false为逆时针。默认为false |
   | motor_speed | int      | 雷达转速，单位Hz，取值范围为5~15Hz。默认值为10Hz             |

   

6. 启动Oradar ros节点

   当环境是ROS时：

   ```shell
   roslaunch oradar_lidar ms200_scan.launch
   或者
   roslaunch oradar_lidar ms200_scan_view.launch (使用rviz显示) 
   ```

   当环境是ROS2时：

   ```
   ros2 launch oradar_lidar ms200_scan.launch.py
   或者
   ros2 launch oradar_lidar ms200_scan_view.launch.py（使用rviz2显示）
   ```

   
