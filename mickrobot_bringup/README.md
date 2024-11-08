#### 环境说明

##### 该节点使用的是ROS2的Galactic版本，使用鱼香ROS的一键安装，按照步骤来即可

```
wget http://fishros.com/install -O fishros && . fishros
```

##### 接着需要安装NAV2的包（可能需要单独安装，以防万一）

```
 sudo apt install ros-$ROS_DISTRO-nav2-*
```

##### ~~然关于ROS2串口驱动的安装，参考CSDN的一篇文章 [ROS2 Serial串口驱动](https://blog.csdn.net/zardforever123/article/details/134227412)~~

~~但是由于ROS2没有再封装串口库serial，因此需要手动安装serial：~~

```
git clone https://github.com/ZhaoXiangBox/serial
cd serial && mkdir build
cmake .. && make
sudo make install
```


#####  编译运行底盘 ROS 节点

1. 下载代码
    ```
    cd ~/ros2_ws/src
    git clone https://github.com/RuPingCen/mick_robot.git 
    ```

2. 返回到工作空间主目录，使用colcon build进行编译:
    ```
    cd ~/ros2_ws
    colcon build
    ```
    
3. 在运行之前，由于节点里面涉及到串口的打开，因此需要先打开串口权限:
    ```
    sudo chmod 777 /dev/ttyUSB0
    ```

4. 最后在编译无报错之后就可以运行了:

    ```
    sourse install/setup.bash
    ros2 launch mick_bringup mickrobotx4.launch.py
    ```



### 通过命令行控制小车

可以将小车垫高，随后在命令行终端里输入以下指令：

```
ros2 topic pub -r 100 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```


### 键盘控制小车底盘

底盘代码包中包含一个键盘控制节点，  'w' 'x' 'a' 'd' 's' 分别表示前 后 左 右 停止。

```
sourse install/setup.bash
ros2 run keyboard keyboard
```



 
