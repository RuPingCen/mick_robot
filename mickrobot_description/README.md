

### 1 安装gazebo环境运行的依赖包

```
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
sudo apt install ros-humble-velodyne-gazebo-plugins
```


### 2 启动gazebo仿真节点

```
ros2 launch mickrobot_description gazebo.launch.py
```

 

默认发布的话题如下：

```
/camera_sensor/camera_info
/camera_sensor/image_raw
/camera_sensor/image_raw/compressed
/camera_sensor/image_raw/compressedDepth
/camera_sensor/image_raw/theora
/clicked_point
/clock
/cmd_vel
/goal_pose
/imu/data
/initialpose
/joint_states
/odom
/parameter_events
/performance_metrics
/robot_description
/rosout
/rslidar_points
/scan
/sensors/gazebo_ros_gps/vel
/sensors/gps
/tf
/tf_static
```



若需要修改默认话题名称，可对mickrobot_stl.urdf中话题名称进行修改

/home/mickrobot/ros2_mick_ws/src/mick_robot/mickrobot_description/urdf/mickrobot_stl.urdf
