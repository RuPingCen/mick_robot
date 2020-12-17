# mick_robot

该代码为麦克纳姆轮和四轮差速ROS底盘的ROS导航节点包，与 mick_robot_chasiss 代码配合使用，更多的信息可以参考[博客-熊猫飞天](https://blog.csdn.net/crp997576280)

该开源项目使用四轮差速小车模型和16线的3D激光雷达作为传感器，导航框架是基于move_base进行修改的。项目对应的中文教程：

[开源自主导航小车MickX4（一）ROS底盘硬件](https://blog.csdn.net/crp997576280/article/details/108290182)

[开源自主导航小车MickX4（二）ROS底盘运动控制](https://blog.csdn.net/crp997576280/article/details/108475154)

[开源自主导航小车MickX4（三）底盘ROS节点](https://blog.csdn.net/crp997576280/article/details/108567732)

[开源自主导航小车MickX4（四）底盘URDF模型](https://blog.csdn.net/crp997576280/article/details/109685109)

开源自主导航小车MickX4（五）gmapping建图

开源自主导航小车MickX4（六）cartography 建图

开源自主导航小车MickX4（七）小车室外导航

开源自主导航小车MickX4（八）总结与展望

![MickX4](https://github.com/RuPingCen/mick_robot_chasiss/raw/master/Reference/mickx4.png)

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

 ```
  roslaunch mick_bringup keyboard.launch
```
## 2.2 启动gmapping建图

 ```
  roslaunch mick_navigation mickx4_gmapping.launch
```
# 3 麦克纳姆轮的底盘 

 ```
  roslaunch mick_bringup mickx4_bringup.launch
```

