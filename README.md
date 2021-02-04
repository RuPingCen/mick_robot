# mick_robot

mick 是一个开源的自主导航小车项目，使用四轮差速小车模型和16线的3D激光雷达作为传感器，导航框架是基于move_base进行修改的。目前支持麦克纳姆轮和四轮差速底盘，该开源项目从搭建机械部分开始，分享底层的嵌入式控制，上层的建图和导航部分，最终实现A点到B的自主导航。 当前地址的代码为麦克纳姆轮和四轮差速ROS底盘的ROS导航节点包，与 mick_robot_chasiss 代码（底层控制代码）配合使用，更多的信息可以参考[博客-熊猫飞天](https://blog.csdn.net/crp997576280)

该开源项目项应的中文教程：

[开源自主导航小车MickX4（一）ROS底盘硬件](https://blog.csdn.net/crp997576280/article/details/108290182)

[开源自主导航小车MickX4（二）ROS底盘运动控制](https://blog.csdn.net/crp997576280/article/details/108475154)

[开源自主导航小车MickX4（三）底盘ROS节点](https://blog.csdn.net/crp997576280/article/details/108567732)

[开源自主导航小车MickX4（四）底盘URDF模型](https://blog.csdn.net/crp997576280/article/details/109685109)

[开源自主导航小车MickX4（五）gmapping建图](https://blog.csdn.net/crp997576280/article/details/109685462)

[开源自主导航小车MickX4（六）cartography 2D 建图](https://blog.csdn.net/crp997576280/article/details/109685590)

[开源自主导航小车MickX4（七）cartography 3D 建图](https://blog.csdn.net/crp997576280/article/details/111600534)

[开源自主导航小车MickX4（八）LeGo-LOAM 室外3D建图](https://blog.csdn.net/crp997576280/article/details/111657554)

[开源自主导航小车MickX4（九）基于move_base 的自主导航框架](https://blog.csdn.net/crp997576280/article/details/113434795)

[开源自主导航小车MickX4（十）总结与展望](https://blog.csdn.net/crp997576280/article/details/113438074#comments_14906009)

该开源框架支持两种类型的底盘，4轮差速底盘和麦克纳姆轮底盘，如下图所示，底盘上运行的STM32程序可以在这个[地址下载](https://github.com/RuPingCen/mick_robot_chasiss)。

![MickX4](https://github.com/RuPingCen/blog/raw/master/mick_robot/fig/mick-fig1.png)

### V1.2 修改日志
  １．在底盘节点中增加使用外部的IMU来矫正里程计的偏航角
  
  ２．cartographer建图配置参数
  
  ３．增加了LeGO-LOAM节点（从官方节点fork）
  
  
### V1.1 修改日志
  １．修改了ROS serial库参数，使得串口读取数据帧更加稳定
  
  ２．增加节点启动时候清零里程计的指令
  
  ３．修复了里程计掉头以后由于方向原因使得位置估算错误的BUG
  
  ４．增加了参数传递功能，可通过launch文件传递参数
  
### v1.0 修改日志
    
    第一次提交

### 代码目录说明

mick_bringup : 为麦克纳姆轮和四轮差速ROS底盘的ROS节点

mick_description :存放模型文件和URDF文件

mick_navigation: move—base导航配置文件
  
# 1 下载安装
 1.1 下载 
 ```
  cd catkin_ws/src

  git clone  https://github.com/RuPingCen/mick_robot.git
```
1.2 安装依赖项 & 编译
```
  cd mick_robot
  
  ./config.sh
  
  cd ~/catkin_ws
  
  catkin_make
```
 
# 2 运行差速底盘 （X4）

## 2.1 启动键盘控制
使用 I J K L 按键控制小车移动，shift按键加速。注意要把遥控器拨到自动档位，使能上位机控制。


 ```
  roslaunch mick_bringup keyboard.launch
```
## 2.2 启动gmapping建图

 ```
  roslaunch mick_navigation mickx4_gmapping.launch
```

## 2.3 cartographer 建图
step1: 启动建图节点（传感器的启动都放在了这个launch文件中）

 ```
  roslaunch mick_navigation mickx4_carto_2D.launch
```

step2: 保存地图

 ```
  rosservice call /write_state ~/cartograph_test.pbstream
```

step3: 用 cartographer 自带的转换节点将.pbstream 文件转化为pgm和yaml文件


 ```
source devel_isolated/setup.bash

rosrun cartographer_ros cartographer_pbstream_to_ros_map -pbstream_filename /home/administrator/cartograph_test.pbstream -map_filestem /home/administrator/cartograph_test
```



# 3 麦克纳姆轮的底盘 

 ```
  roslaunch mick_bringup mickx4_bringup.launch
```

