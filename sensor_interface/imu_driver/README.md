
# IMU_Driver

目前支持 第七实验室的IMU和维特智能的IMU设备 WTGAHRS1 （非官方驱动，为方便个人适用而写）

 ## 1.1 下载与配置

 1. 安装依赖项
 
    sudo apt-get install ros-kinetic-serial
    
 2. cd catkin_ws/src
 
 3. git clone  https://github.com/RuPingCen/IMU_Driver.git

 4. catkin_make
 
 ## 1.2 启动维特智能的IMU设备
 
     roslaunch imu_driver wit_imu.launch
     
  ## 1.3 启动第七实验室的IMU设备
 
     roslaunch imu_driver seven_lab.launch
