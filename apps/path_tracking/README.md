# Path Tracking Package

ROS2轨迹跟踪功能包，实现了Pure Pursuit和Stanley两种经典控制算法。

## 1、功能概述

- **路径发布器**: 发布正弦曲线参考轨迹（沿X方向20m，Y方向振幅±3m）
- **Pure Pursuit控制器**: 基于前视距离的轨迹跟踪算法
- **Stanley控制器**: 基于横向误差和航向误差的轨迹跟踪算法

---
## 2、使用方法

### 2.1  编译功能包
```bash
#先启动仿真
ros2 launch mickrobot_description gazebo.launch.py
```

```bash
cd ~/ros2_mickrobot_ws
colcon build --packages-select path_tracking
source install/setup.bash
```

### 2.2 启动Pure Pursuit控制器

```bash
ros2 launch path_tracking pure_pursuit.launch.py
```

### 2.3 启动Stanley控制器

```bash
ros2 launch path_tracking stanley.launch.py
```

### 2.4 仅启动路径发布器

```bash
ros2 launch path_tracking path_only.launch.py
```

### 2.5 话题说明

- **订阅话题**:
  - `/odom` (nav_msgs/Odometry): 机器人里程计信息
  - `/reference_path` (nav_msgs/Path): 参考轨迹路径

- **发布话题**:
  - `/cmd_vel` (geometry_msgs/Twist): 速度控制命令
  - `/reference_path` (nav_msgs/Path): 参考轨迹路径

### 2.6 参数配置

#### 2.6.1 Pure Pursuit 当前参数

```yaml
lookahead_distance: 0.6          # 前视距离（米）
linear_velocity: 0.6             # 线速度（米/秒）
max_angular_velocity: 3.5        # 最大角速度（弧度/秒）
min_linear_velocity: 0.45        # 最小线速度（米/秒）
position_tolerance: 0.12         # 位置到达容差（米）
yaw_tolerance: 0.05              # 朝向到达容差（弧度）
```

#### 2.6.2Stanley 当前参数

```yaml
k_e: 1.5                         # 横向误差增益
k_v: 0.1                         # 速度增益
linear_velocity: 0.6             # 线速度（米/秒）
max_angular_velocity: 3.0        # 最大角速度（弧度/秒）
wheelbase: 0.52                  # 轮距（米）- 必须准确！
lookahead_gain: 0.3              # 前视增益
```

### 2.7 单独运行节点

```bash
# 路径发布器
ros2 run path_tracking circle_path_publisher

# Pure Pursuit控制器
ros2 run path_tracking pure_pursuit_controller

# Stanley控制器
ros2 run path_tracking stanley_controller
```

### 2.8 可视化

在RViz中添加以下显示项：
- `/reference_path` (Path): 显示参考轨迹
- `/odom` (Odometry): 显示机器人位置



## 3 算法参数说明

### 3.1 🎯 Pure Pursuit 算法关键参数说明

| 参数 | 推荐值 | 作用 |
|------|--------|------|
| `lookahead_distance` | 0.6m | 前视距离，越小转弯越急，越大路径越平滑 |
| `linear_velocity` | 0.6m/s | 线速度，速度越快需要更大的前视距离 |
| `max_angular_velocity` | 3.5rad/s | 最大角速度，限制最大转弯速度 |

---

### 3.2 🎯 Stanley 算法关键参数说明

| 参数 | 推荐值 | 作用 |
|------|--------|------|
| `k_e` | 1.5 | 横向误差增益，越大纠偏越快（太大会震荡） |
| `k_v` | 0.1 | 速度增益，越小低速纠偏越强 |
| `wheelbase` | 0.52m | 轮距，必须与机器人实际轮距一致 |
| `lookahead_gain` | 0.3 | 前视增益，防止"追着尾巴跑" |
| `linear_velocity` | 0.6m/s | 线速度 |
| `max_angular_velocity` | 3.0rad/s | 最大角速度 |

---

### 3.3  🆚 两种算法对比

| 特性 | Pure Pursuit | Stanley |
|------|--------------|---------|
| **原理** | 追踪前视点 | 纠正航向+横向误差 |
| **路径平滑度** | ⭐⭐⭐⭐⭐ 非常平滑 | ⭐⭐⭐⭐ 较平滑 |
| **跟踪精度** | ⭐⭐⭐ 中等 | ⭐⭐⭐⭐⭐ 很高 |
| **计算复杂度** | ⭐⭐ 简单 | ⭐⭐⭐ 中等 |
| **参数调节难度** | ⭐⭐ 简单 | ⭐⭐⭐⭐ 较复杂 |
| **适用场景** | 高速、平滑路径 | 精确跟踪、复杂路径 |
| **弯道表现** | 可能抄近道 | 紧贴轨迹 |
| **速度适应性** | 需要调整前视距离 | 公式自动适应 |
