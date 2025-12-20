import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
import numpy as np

class StanleyController(Node):
    def __init__(self):
        super().__init__("stanley_controller")

        # 订阅里程计和参考路径
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Path, "/reference_path", self.path_callback, 10)

        # 发布速度命令
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Stanley 参数
        self.declare_parameter("k_e", 1.5)                  # 横向误差增益，提高以增强纠偏能力
        self.declare_parameter("k_v", 0.1)                  # 速度相关增益，降低以减小速度影响
        self.declare_parameter("linear_velocity", 0.6)      # 线速度，提高速度
        self.declare_parameter("max_angular_velocity", 3.0) # 最大角速度，大幅提高
        self.declare_parameter("wheelbase", 0.52)           # 轮距，根据URDF修正（0.26*2）
        self.declare_parameter("lookahead_gain", 0.3)       # 前视增益

        self.k_e = self.get_parameter("k_e").value
        self.k_v = self.get_parameter("k_v").value
        self.linear_vel = self.get_parameter("linear_velocity").value
        self.max_angular_vel = self.get_parameter("max_angular_velocity").value
        self.wheelbase = self.get_parameter("wheelbase").value
        self.lookahead_gain = self.get_parameter("lookahead_gain").value

        # 状态变量
        self.current_pose = None
        self.current_yaw = 0.0
        self.reference_path = None
        self.current_velocity = 0.0  # 添加实际速度跟踪

        # 控制循环
        self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info(f"Stanley controller started (k_e={self.k_e}, v={self.linear_vel}m/s)")

    def odom_callback(self, msg):
        """接收里程计数据"""
        self.current_pose = msg.pose.pose.position
        # 从四元数计算yaw角
        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 获取实际速度
        vel = msg.twist.twist.linear
        self.current_velocity = math.sqrt(vel.x**2 + vel.y**2)

    def path_callback(self, msg):
        """接收参考路径"""
        if len(msg.poses) > 0:
            self.reference_path = msg
            self.get_logger().info(f"Received path with {len(msg.poses)} points", once=True)

    def find_nearest_point(self):
        """找到路径上前方的最近点（带前视）"""
        if self.reference_path is None or self.current_pose is None:
            return None, None, None

        min_dist = float('inf')
        nearest_idx = 0
        
        current_x = self.current_pose.x
        current_y = self.current_pose.y

        # 首先找到最近的路径点
        for idx, pose in enumerate(self.reference_path.poses):
            path_x = pose.pose.position.x
            path_y = pose.pose.position.y
            
            dist = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                nearest_idx = idx

        # 在最近点基础上向前查找一小段距离（前视）
        lookahead_dist = self.lookahead_gain * self.linear_vel  # 动态前视距离
        lookahead_idx = nearest_idx
        
        # 从最近点开始向前搜索
        accumulated_dist = 0.0
        for i in range(nearest_idx, len(self.reference_path.poses) - 1):
            p1 = self.reference_path.poses[i].pose.position
            p2 = self.reference_path.poses[i + 1].pose.position
            segment_dist = math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)
            accumulated_dist += segment_dist
            
            if accumulated_dist >= lookahead_dist:
                lookahead_idx = i + 1
                break
        
        target_pose = self.reference_path.poses[lookahead_idx]
        
        # 计算到目标点的距离（横向误差）
        target_dist = math.sqrt((target_pose.pose.position.x - current_x)**2 + 
                               (target_pose.pose.position.y - current_y)**2)
        
        # 计算路径点的朝向（yaw）
        quat = target_pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        path_yaw = math.atan2(siny_cosp, cosy_cosp)

        return target_pose.pose.position, path_yaw, target_dist

    def compute_control(self, nearest_point, path_yaw, cross_track_error):
        """计算Stanley控制命令（优化版）"""
        if nearest_point is None:
            return 0.0, 0.0

        # 1. 计算航向误差
        heading_error = path_yaw - self.current_yaw
        
        # 归一化到 [-pi, pi]
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        # 2. 计算横向误差项
        # 计算机器人前轮中心点（用于更准确的误差计算）
        front_x = self.current_pose.x + self.wheelbase * 0.5 * math.cos(self.current_yaw)
        front_y = self.current_pose.y + self.wheelbase * 0.5 * math.sin(self.current_yaw)
        
        # 重新计算到路径的横向误差
        dx = nearest_point.x - front_x
        dy = nearest_point.y - front_y
        
        # 计算误差的符号（使用前轮位置）
        cross_product = -dx * math.sin(self.current_yaw) + dy * math.cos(self.current_yaw)
        actual_cross_error = math.sqrt(dx**2 + dy**2)
        cross_track_error_signed = actual_cross_error if cross_product > 0 else -actual_cross_error

        # Stanley公式：δ = θ_e + arctan(k_e * e / (k_v + v))
        # 使用实际速度或目标速度的较小值
        effective_velocity = max(abs(self.current_velocity), 0.1)  # 避免除零
        crosstrack_term = math.atan2(self.k_e * cross_track_error_signed, 
                                      self.k_v + effective_velocity)

        # 3. 计算转向角
        steering_angle = heading_error + crosstrack_term
        
        # 归一化
        steering_angle = math.atan2(math.sin(steering_angle), math.cos(steering_angle))

        # 4. 转换为角速度（差速模型）- 优化转换系数
        # 对于差速机器人，使用更直接的转换
        angular_vel = steering_angle * (self.linear_vel / self.wheelbase) * 2.5  # 增加系数
        
        # 限制角速度
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)

        return self.linear_vel, angular_vel

    def control_loop(self):
        """控制循环"""
        if self.current_pose is None or self.reference_path is None:
            return

        # 找到最近点和横向误差
        nearest_point, path_yaw, cross_track_error = self.find_nearest_point()
        
        if nearest_point is None:
            return

        # 计算控制命令
        linear_vel, angular_vel = self.compute_control(nearest_point, path_yaw, cross_track_error)

        # 发布速度命令
        cmd = Twist()
        cmd.linear.x = linear_vel
        cmd.angular.z = angular_vel
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = StanleyController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
