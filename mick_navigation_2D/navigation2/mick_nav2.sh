#!/bin/bash

# Function to handle termination
terminate() {
  echo "终止所有后台进程..."
  kill $ros2_launch_pid $ekf_node_pid $imu_encoder_mix_pid $nav2_launch_pid $controller_launch_pid
  wait $ros2_launch_pid $ekf_node_pid $imu_encoder_mix_pid $nav2_launch_pid $controller_launch_pid
  echo "所有进程已终止。"
}

# Trap Ctrl+C (SIGINT)
trap terminate SIGINT

# 检查ROS 2环境
if [ -z "$ROS_DISTRO" ]; then
  echo "请先设置好ROS 2环境变量，例如'source /opt/ros/galactic/setup.bash'"
  exit 1
fi

echo "----------"
# 启动 ROS 2 launch 文件
echo "启动 mickrobot "
gnome-terminal -- bash -c "cd /home/mickrobot/ros2_ws; source install/setup.bash; ros2 launch mickrobot_bringup mickrobotx4.launch.py; exec bash"
echo "mickrobot 已启动 "
# 等待5秒
sleep 2
echo "----------"

# 启动 雷达 节点
echo "启动 雷达 "
gnome-terminal -- bash -c "cd /home/mickrobot/ros2_ws; source install/setup.bash; ros2 launch sllidar_ros2 sllidar_a2m7_launch.py ; exec bash"
echo "雷达 已启动 "
sleep 2
echo "----------"

# 启动 imu 节点
echo "启动 IMU"
gnome-terminal -- bash -c "cd /home/mickrobot/ros2_ws; source install/setup.bash; ros2 launch mick_imu_node imu_100D2.launch.py; exec bash"
echo "IMU 已启动 "
sleep 2
echo "----------"

#启动 tf变换 节点
echo "启动 tf变换"
gnome-terminal -- bash -c "cd /home/mickrobot/ros2_ws; source install/setup.bash; ros2 launch odom_tf odom.launch.py; exec bash"
echo "TF变换 已启动 "
sleep 2
echo "----------"

#启动 EKF 节点
echo "启动 EKF"
gnome-terminal -- bash -c "cd /home/mickrobot/ros2_ws; source install/setup.bash; ros2 launch robot_localization ekf.launch.py; exec bash"
echo "EKF 已启动 "
sleep 2
echo "----------"

# 启动 Nav2 bringup 节点
echo "启动 Nav2 bringup "
gnome-terminal -- bash -c "cd /home/mickrobot/ros2_ws; source install/setup.bash; ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=False map:=/home/mickrobot/ros2_ws/src/navigation2/maps/map_out.yaml; exec bash"
echo "nav2_bringup 已启动 "
sleep 2
echo "----------"

#启动 Rviz2 
echo "启动 Rviz2"
gnome-terminal -- bash -c "cd /home/mickrobot/ros2_ws; source install/setup.bash; rviz2 -d /home/mickrobot/ros2_ws/src/navigation2/rviz/navigation2.rviz; exec bash"
echo "Rviz2 已启动"
echo "----------"

# 等待所有后台进程完成
wait $ros2_launch_pid $ekf_node_pid $imu_encoder_mix_pid $nav2_launch_pid