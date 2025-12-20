import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist
import numpy as np

class PurePursuitController(Node):
    def __init__(self):
        super().__init__("pure_pursuit_controller")

        # 订阅里程计和参考路径
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.create_subscription(Path, "/reference_path", self.path_callback, 10)

        # 发布速度命令
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Pure Pursuit 参数
        self.declare_parameter("lookahead_distance", 0.6)  # 前视距离减小，增大曲率计算
        self.declare_parameter("linear_velocity", 0.6)     # 线速度
        self.declare_parameter("max_angular_velocity", 3.5) # 最大角速度提高
        self.declare_parameter("min_linear_velocity", 0.45)  # 最小线速度提高
        self.declare_parameter("position_tolerance", 0.12)   # 位置到达容差
        self.declare_parameter("yaw_tolerance", 0.05)        # 朝向到达容差

        self.lookahead_dist = self.get_parameter("lookahead_distance").value
        self.linear_vel = self.get_parameter("linear_velocity").value
        self.max_angular_vel = self.get_parameter("max_angular_velocity").value
        self.min_linear_vel = self.get_parameter("min_linear_velocity").value
        self.position_tol = self.get_parameter("position_tolerance").value
        self.yaw_tol = self.get_parameter("yaw_tolerance").value

        # 状态变量
        self.current_pose = None
        self.current_yaw = 0.0
        self.reference_path = None
        
        # 状态机: APPROACH(接近轨迹) -> ALIGN(调整朝向) -> TRACKING(跟踪)
        self.state = "APPROACH"
        self.target_point_idx = 0

        # 控制循环
        self.create_timer(0.05, self.control_loop)  # 20Hz

        self.get_logger().info(f"Pure Pursuit controller started (lookahead={self.lookahead_dist}m, v={self.linear_vel}m/s)")
        self.get_logger().info("State: APPROACH - Moving to trajectory")

    def odom_callback(self, msg):
        """接收里程计数据"""
        self.current_pose = msg.pose.pose.position
        # 从四元数计算yaw角
        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def path_callback(self, msg):
        """接收参考路径"""
        if len(msg.poses) > 0:
            self.reference_path = msg
            self.get_logger().info(f"Received path with {len(msg.poses)} points", once=True)

    def find_nearest_point_on_path(self):
        """找到路径上最近的点"""
        if self.reference_path is None or self.current_pose is None:
            return None, None, None

        min_dist = float('inf')
        nearest_idx = 0
        
        current_x = self.current_pose.x
        current_y = self.current_pose.y

        # 找到最近的路径点
        for idx, pose in enumerate(self.reference_path.poses):
            path_x = pose.pose.position.x
            path_y = pose.pose.position.y
            
            dist = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                nearest_idx = idx

        nearest_pose = self.reference_path.poses[nearest_idx]
        
        # 计算路径点的朝向（yaw）
        quat = nearest_pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        path_yaw = math.atan2(siny_cosp, cosy_cosp)

        return nearest_idx, min_dist, path_yaw

    def approach_trajectory(self):
        """状态1: 接近轨迹点"""
        nearest_idx, dist, path_yaw = self.find_nearest_point_on_path()
        
        if nearest_idx is None:
            return
        
        target_pose = self.reference_path.poses[nearest_idx]
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        
        # 计算到目标点的方向
        dx = target_x - self.current_pose.x
        dy = target_y - self.current_pose.y
        target_angle = math.atan2(dy, dx)
        
        # 计算角度差
        angle_diff = target_angle - self.current_yaw
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        cmd = Twist()
        
        # 如果到达轨迹附近，切换到对齐状态
        if dist < self.position_tol:
            self.state = "ALIGN"
            self.target_point_idx = nearest_idx
            self.get_logger().info(f"State: ALIGN - Aligning orientation (dist={dist:.3f}m)")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # 先转向目标点，然后前进
            if abs(angle_diff) > 0.2:  # 如果角度差大于约11度
                cmd.linear.x = 0.0
                cmd.angular.z = np.clip(angle_diff * 2.0, -self.max_angular_vel, self.max_angular_vel)
            else:
                cmd.linear.x = self.linear_vel * 0.6  # 接近时适当降速
                cmd.angular.z = np.clip(angle_diff * 2.5, -self.max_angular_vel, self.max_angular_vel)
        
        self.cmd_pub.publish(cmd)

    def align_orientation(self):
        """状态2: 调整朝向"""
        if self.reference_path is None or self.target_point_idx >= len(self.reference_path.poses):
            return
        
        target_pose = self.reference_path.poses[self.target_point_idx]
        
        # 获取目标朝向
        quat = target_pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        target_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # 计算朝向误差
        yaw_error = target_yaw - self.current_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        
        cmd = Twist()
        
        # 如果朝向对齐，切换到跟踪状态
        if abs(yaw_error) < self.yaw_tol:
            self.state = "TRACKING"
            self.get_logger().info(f"State: TRACKING - Following trajectory (yaw_error={abs(yaw_error):.3f}rad)")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            # 原地旋转调整朝向
            cmd.linear.x = 0.0
            cmd.angular.z = np.clip(yaw_error * 1.5, -self.max_angular_vel, self.max_angular_vel)
        
        self.cmd_pub.publish(cmd)

    def find_lookahead_point(self):
        """找到前视点（用于跟踪状态）"""
        if self.reference_path is None or self.current_pose is None:
            return None

        current_x = self.current_pose.x
        current_y = self.current_pose.y

        # 首先找到最近的路径点作为起点
        min_dist = float('inf')
        nearest_idx = 0
        
        for idx, pose in enumerate(self.reference_path.poses):
            path_x = pose.pose.position.x
            path_y = pose.pose.position.y
            dist = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                nearest_idx = idx

        # 从最近点开始，沿着路径向前搜索前视点
        # 搜索范围：从最近点向前查找
        search_start = nearest_idx
        search_range = len(self.reference_path.poses)
        
        lookahead_point = None
        best_idx = nearest_idx
        
        for i in range(search_range):
            idx = (search_start + i) % len(self.reference_path.poses)
            pose = self.reference_path.poses[idx]
            path_x = pose.pose.position.x
            path_y = pose.pose.position.y
            
            dist = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
            
            # 找到第一个距离大于等于前视距离的点
            if dist >= self.lookahead_dist:
                lookahead_point = (path_x, path_y)
                best_idx = idx
                break
        
        # 如果没找到，使用距离最接近前视距离的点
        if lookahead_point is None:
            min_diff = float('inf')
            for i in range(search_range):
                idx = (search_start + i) % len(self.reference_path.poses)
                pose = self.reference_path.poses[idx]
                path_x = pose.pose.position.x
                path_y = pose.pose.position.y
                
                dist = math.sqrt((path_x - current_x)**2 + (path_y - current_y)**2)
                diff = abs(dist - self.lookahead_dist)
                
                if diff < min_diff:
                    min_diff = diff
                    lookahead_point = (path_x, path_y)
                    best_idx = idx
        
        return lookahead_point

    def compute_control(self, lookahead_point):
        """计算Pure Pursuit控制命令（用于跟踪状态）"""
        if lookahead_point is None:
            return 0.0, 0.0

        # 将前视点转换到机器人坐标系
        dx = lookahead_point[0] - self.current_pose.x
        dy = lookahead_point[1] - self.current_pose.y
        
        # 转换到机器人局部坐标系
        local_x = math.cos(self.current_yaw) * dx + math.sin(self.current_yaw) * dy
        local_y = -math.sin(self.current_yaw) * dx + math.cos(self.current_yaw) * dy

        # Pure Pursuit 公式
        # 曲率 k = 2 * y / L^2
        L = math.sqrt(local_x**2 + local_y**2)
        if L < 0.01:
            return self.linear_vel, 0.0

        curvature = 2.0 * local_y / (L * L)
        
        # 自适应速度：根据曲率调整速度（优化参数）
        # 进一步减小曲率影响，保持高速过弯
        curvature_factor = 1.0 / (1.0 + abs(curvature) * 0.5)  # 从0.8降到0.5
        adaptive_vel = self.min_linear_vel + (self.linear_vel - self.min_linear_vel) * curvature_factor
        
        # 确保弯道速度不低于最大速度的70%
        adaptive_vel = max(adaptive_vel, self.linear_vel * 0.7)
        
        # 角速度 = v * k
        angular_vel = adaptive_vel * curvature
        
        # 限制角速度
        angular_vel = np.clip(angular_vel, -self.max_angular_vel, self.max_angular_vel)

        return adaptive_vel, angular_vel

    def control_loop(self):
        """控制循环 - 状态机"""
        if self.current_pose is None or self.reference_path is None:
            return

        if self.state == "APPROACH":
            # 状态1: 接近轨迹
            self.approach_trajectory()
            
        elif self.state == "ALIGN":
            # 状态2: 调整朝向
            self.align_orientation()
            
        elif self.state == "TRACKING":
            # 状态3: Pure Pursuit跟踪
            lookahead_point = self.find_lookahead_point()
            linear_vel, angular_vel = self.compute_control(lookahead_point)

            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
