# mick_robot

mick_robot 是一个开源的自主导航小车项目名称，使用四轮差速小车和32线3D激光雷达作为传感器，2D导航框架是基于move_base进行修改，主要面向室内环境。3D导航框架对autoware代码就行修改和裁剪得到，面向室外环境。目前支持麦克纳姆轮和四轮差速底盘，该开源项目从机械装配部分开始，分享小车底层的嵌入式控制，上层的建图和导航部分，最终实现A点到B的自主导航。 当前master分支代码基于ROS2实现，分支ROS1为早期版本。[更新说明](https://github.com/RuPingCen/mick_robot_chasiss/blob/master/Update.md)

使用该导航代码需要自备一台实验小车，如有自行搭建小车底盘的想法可以参考我们另外一个开源项目： [mick_robot_chassis](https://github.com/RuPingCen/mick_robot_chassis) ，更多的信息可以参考[博客-熊猫飞天](https://blog.csdn.net/crp997576280)

**[mick_robot](https://github.com/RuPingCen/mick_robot)**  包含移动机器人定位和导航部分算法开发

[**mick_robot_chasiss**](https://github.com/RuPingCen/mick_robot_chassis)  包含移动机器人底盘的开发（机械设计/底层的嵌入式控制/PCB设计）



### 代码目录说明

- **mick_bringup** :  为麦克纳姆轮和四轮差速ROS底盘的ROS节点
- **mick_description** : 存放模型文件和URDF文件
- **mick_navigation**:  move base导航配置文件

 # 1 Gazebo仿真环境下运行小车

安装gazebo环境运行的依赖包

```
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

启动gazebo仿真节点

```
ros2 launch mickrobot_description gazebo.launch.py
```
启动后节点将发布以下话题

```
/camera_sensor/camera_info
/camera_sensor/image_raw
/cmd_vel
/imu
/joint_states
/odom
/parameter_events
/performance_metrics
/robot_description
/rosout
/scan            
/sensors/gazebo_ros_gps/vel
/sensors/gps
/tf
/tf_static
```

<img src="mickrobot_description/fig/gazebo_fig2.gif" alt="gazebo_fig1" style="zoom:100%;" />

# 2 运行实体小车

## 2.1 键盘控制小车

### 2.1.1 下载节点代码
 ```
  mkdir -p ros2_ws/src
  cd ros2_ws/src
  git clone  https://github.com/RuPingCen/mick_robot.git
 ```
### 1.2 安装依赖项 & 编译
```
colcon build
```

### 1.3 启动小车

打开串口权限:

```
sudo chmod 777 /dev/ttyUSB0
```

启动小车节点

```
ros2 run mick_bringup mick_bringup
```

### 1.4 启动键盘控制

新建一个终端向 小车命令接收话题 /cmd_vel 话题发送数据


 ```
 ros2 topic pub -r 100 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
 ```
利用键盘控制节点控制小车，在目录 **sensor_interface**目录下有键盘控制节点 keyboard，启动该节点利用键盘方向件以及 w x a d s 按键 分别控制小车前、后、左、右、停止。


 ```
ros2 run keyboard keyboard
 ```

## 2.2 运行建图节点





## 2.3 运行导航节点





