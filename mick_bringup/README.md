# mick_robot_bringup
该代码为麦克纳姆轮ROS底盘的ROS节点，与mick_robot_chasiss代码配合使用，更多的信息可以参考博客地址：https://blog.csdn.net/crp997576280/article/details/102026459
### V1.1 修改日志
  １．修改了ROS serial库参数，使得串口读取数据帧更加稳定
  
  ２．增加节点启动时候清零里程计的指令
  
  ３．修复了里程计掉头以后由于方向原因使得位置估算错误的BUG
  
  ４．增加了参数传递功能，可通过launch文件传递参数
  
### Ｖ1.0 修改日志
  第一次提交
  
# 1、下载安装
 1.安装依赖项
 
    sudo apt-get install ros-kinetic-serial
    
 2. cd catkin_ws/src
 
 3.  git clone  https://github.com/RuPingCen/mick_robot_bringup.git

 4. catkin_make
