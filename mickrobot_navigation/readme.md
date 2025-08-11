1.使用gmapping算法建图

ros2 launch slam_gmapping_sim slam_gmapping.launch.py 

如果不更换world,则不需要建图

2.启动gazebo

ros2 launch mickrobot_description gazebo.launch.py 

3.启动导航仿真

ros2 launch navigation_sim nav2.launch.py