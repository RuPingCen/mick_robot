# ROS小车节点

仅小车底盘节点（**mickrobot_bringup**）同时支持ROS1和ROS2环境。

## 1.环境说明

##### 该节点使用的是ROS2的Galactic版本，使用鱼香ROS的一键安装，按照步骤来即可

```
wget http://fishros.com/install -O fishros && . fishros
```

##### 接着需要安装NAV2的包（可能需要单独安装，以防万一）

```
 sudo apt install ros-$ROS_DISTRO-nav2-*
```

若提示serial库没有安装，可将thridpart_lib文件夹下的serial库安装安装

```
cp thridpart_lib/serial.zip ~/
cd ~
unzip serial
cd serial && mkdir build
cmake .. && make
sudo make install
```


## 2.  ROS2 节点
### 2.1 编译ROS2代码

1. 下载代码
    ```
    cd ~/ros2_ws/src
    git clone https://github.com/RuPingCen/mick_robot.git 
    ```

2. 返回到工作空间主目录，使用colcon build进行编译:
    ```
    cd ~/ros2_ws
    colcon build
    sourse install/setup.bash
    ```
    
3. 在运行之前，由于节点里面涉及到串口的打开，因此需要先打开串口权限:
    ```
    sudo chmod 777 /dev/ttyUSB0
    ```

4. 最后在编译无报错之后就可以运行了:

    ```
    ros2 launch mick_bringup mickrobotx4.launch.py
    ```

### 2.2 通过命令行控制小车

可以将小车垫高，随后在命令行终端里输入以下指令：

```
ros2 topic pub -r 100 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```


### 2.3 键盘控制小车底盘

底盘代码包中包含一个键盘控制节点，  'w' 'x' 'a' 'd' 's' 分别表示前 后 左 右 停止。

```
sourse install/setup.bash
ros2 run keyboard keyboard
```

## 3. ROS1节点

### 3.1 编译ROS1代码

 **step1:** 将ROS_Node中的mick_bringup目标代码拷贝到ROS工作空间进行编译

```shell
cp -r mick_bringup ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_make
source devel/setup.bash
```

**step2**： 通过USB转232串口线连接电脑和小车控制板，

修改串口权限（可以查看设备的ID号 ls /dev/ttyUSB*）

```
sudo chmod 766 /dev/ttyUSB0
```

 或者根据串口设备的标识符去设置串口别名

```
cd mick_bringup/scripts
sudo cp ultrasonic.rules /etc/udev/rules.d/
```

把文件拷贝到/etc/udev/rules.d目录下就可以使用 /dev/mick替代 /dev/ttyUSB0 来访问模块了

**step3:** 启动ROS节点

```shell
roslaunch mick_bringup mickrobotx4_ROS1.launch
```

小车节点启动以后可以通过rostopic list命令查看到该ROS节点会对外发布如下Topic

<div align=center>
<img src="README.assets/ROS-node-1-1718377977781-1.png" alt="ROS-node-1" />
</div>


其中：

- /mickrobot/chassis/Imu     对外发布小车自身IMU测量数据
- /mickrobot/chassis/odom  小车里程计数据
- /mickrobot/chassis/odom/path   小车里程计数据对应的路径（默认不发布）
- /mickrobot/rc_remotes/joy  遥控器数据（默认不发布）
- /mickrobot/chassis/cmd_vel    小车控制命令接收话题



mickrobot_v3 launch文件内容如下：

```xml
<launch>
  <node pkg="mick_bringup" type="mickx4_bringup_v3" name="mickrobot" output="screen">

	  <param name="dev" value="/dev/ttyUSB0" type="str" />
	  <param name="baud" value="115200" type="int" />
	  <param name="time_out" value="1000" type="int" />
	  <param name="hz" value="100" type="int" />

	  <param name="chassis_type" value="0" type="int" /> <!--0: 差速底盘  1: 麦克纳姆轮底盘 2:阿卡曼转向  3全向-->

	  <param name="sub_cmdvel_topic" value="chassis/cmd_vel" type="str" />
	  <param name="pub_odom_topic" value="chassis/odom" type="str" />
	  <param name="pub_imu_topic" value="chassis/Imu" type="str" />
	  
	  <param name="is_pub_path" value="1" type="int" /> <!--0: 不发布底盘轨迹  1: 发布 -->
	</node>
</launch>
```

### 3.2 通过命令行控制小车

新建终端，通过ROS话题向  **/mickrobot/chassis/cmd_vel** 话题发布数据 控制小车移动（**注意 将遥控器左上角拨码开关拨到最上，表示开启自动驾驶模式**）

```shell
rostopic pub /mickrobot/chassis/cmd_vel -r 10 geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.1" 
```

角速度方向： 逆时针为正，速度方向：车头方向为x方向。

### 3.3 键盘控制小车底盘

这里借助turtlebot3提供的键盘控制节点，检验小车与ROS节点通讯是否正常

注意：需要安装  **sudo apt-get install ros-noetic-turtlebot3-teleop** 包

```shell
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_bringup turtlebot3_model.launch
```



