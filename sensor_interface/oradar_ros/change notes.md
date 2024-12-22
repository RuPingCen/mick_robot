# Oradar_ROS

## version
oradar_ROS_1.0.2 

## modify
1. 解决对串口重复打开问题
2. 阻塞式获取点云数据，采用信号方式进行阻塞，降低CPU占用率

## version
oradar_ROS_1.0.3 

## modify
1. 针对阻塞式接口功能：初始化pthread相关变量，给pthread条件变量加锁，解决了获取点云数据时间不均匀问题

## version
oradar_ROS_1.0.4

## modify
1. 修改串口重复打开问题，兼容嵌入式linux平台

## version
oradar_ROS_1.0.5

## modify
1. 添加 C API复用C++接口类， 并添加C API测试示例
2. 添加设置雷达电机转速、进入待机状态、测距状态接口

## version
oradar_ROS_1.0.6

## modify
1. 解决设置电机转速时工作状态不同步问题

## version
oradar_ROS_1.1.0

## modify
1. 添加ROS2支持
2. 添加windows系统支持

## version
oradar_ROS_1.2.0

## modify
1. 解决概率出现设置测距模式雷达电机无法启动问题
2. 解决先调用GrabFullScan后调用GrabFullScanBlocking，点云获取异常问题
3. 解决串口关闭下能继续获取点云数据问题

## version
oradar_ROS_1.3.0

## modify
1. 解决ROS程序设置转速不成功问题

## version
oradar_ROS_1.3.1

## modify
1. 添加一圈点云数据滤波功能
2. 添加Ubuntu 18.04 ROS2 dashing和eloquent版本的支持

## version
oradar_ROS_1.3.2

## modify
1. 添加双边滤波、拖尾滤波、强度值滤波功能
2. 添加获取固件版本功能

## version
oradar_ROS_1.3.3

## modify
1. 添加获取雷达设备SN号功能
2. 修改上下部组固件版本号功能

## version
oradar_ROS_1.3.4

## modify
1. 解决ROS运行后，在RVIZ软件上点云显示晃动问题