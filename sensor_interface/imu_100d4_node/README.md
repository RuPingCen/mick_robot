 ## 100D4 IMU 

参考官方驱动修改，将100D4 IMU输出数据变换到ENU 坐标系与Xsens MTI-30输出数据保持一致。

### 1. 使用方法

(1).赋予串口权限

```
sudo chmod 777 /dev/ttyUSB0
```

(2). 启动IMU节点

```
ros2 launch mick_imu_node imu_100D4.launch.py
```

 

### 2. 参数说明

- **port** ： 串口号（默认值为 “/dev/ttyUSB0” ）
- **baud**： 波特率型号（默认值为 115200 ）
- **model**： IMU型号（默认值为 “100D4” ）
- **frame_id**： IMU话题输出数据frame id（默认值为 “imu” ）
-  **gravity**： 当地重力参数（默认值为 9.81 ），若希望在话题上需要输出归一化的重力值则可将此参数设置为1
-  **use_magnetic**： 是否使用100D4的磁力计修正航向角（默认值为0，表示不使用 ）

 

​        

 
