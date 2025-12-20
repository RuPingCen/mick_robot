import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CirclePathPublisher(Node):
    def __init__(self):
        super().__init__("circle_path_publisher")

        self.pub = self.create_publisher(Path, "/reference_path", 1)

        # 路径参数（要和Gazebo里一致）
        self.length = 20.0    # 总长度 20m
        self.amplitude = 3.0  # 振幅 3m
        self.frequency = 1.0  # 频率（完整波动次数）
        self.N = 200          # 点数
        self.frame_id = "odom"

        self.path_msg = self.make_sine_path()
        self.timer = self.create_timer(1.0, self.publish_path)

        self.get_logger().info("Sine curve path publisher started (X direction).")

    def make_sine_path(self):
        path = Path()
        path.header.frame_id = self.frame_id

        for i in range(self.N):
            # X方向线性增长
            x = (i / float(self.N)) * self.length
            
            # Y方向正弦曲线
            y = self.amplitude * math.sin(2.0 * math.pi * self.frequency * i / float(self.N))

            ps = PoseStamped()
            ps.header.frame_id = self.frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = 0.0

            # 计算切线方向的朝向
            if i < self.N - 1:
                x_next = ((i + 1) / float(self.N)) * self.length
                y_next = self.amplitude * math.sin(2.0 * math.pi * self.frequency * (i + 1) / float(self.N))
                dx = x_next - x
                dy = y_next - y
                yaw = math.atan2(dy, dx)
            else:
                # 最后一个点使用前一个点的方向
                x_prev = ((i - 1) / float(self.N)) * self.length
                y_prev = self.amplitude * math.sin(2.0 * math.pi * self.frequency * (i - 1) / float(self.N))
                dx = x - x_prev
                dy = y - y_prev
                yaw = math.atan2(dy, dx)

            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)

            path.poses.append(ps)

        return path

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.path_msg)

def main():
    rclpy.init()
    node = CirclePathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
