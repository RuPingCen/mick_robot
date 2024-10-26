#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardReader:
    def __init__(self):
        self.settings = termios.tcgetattr(sys.stdin)

    def __enter__(self):
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *args):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def get_key(self):
        """读取按键并处理多字符序列（如箭头键）。"""
        key = sys.stdin.read(1)
        if key == '\x1b':  # 检测到转义序列的开始
            key += sys.stdin.read(2)  # 读取剩余的两个字符
        return key

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed_level = 1  # 默认速度等级
        self.current_twist = Twist()
        self.max_speed = 1.0  # 最高速度

        # 定义按键与方向的映射，包括箭头键
        self.key_mapping = {
            'w': (1, 0),      # 前进
            'x': (-1, 0),     # 后退
            'a': (0, 1),      # 左转
            'd': (0, -1),     # 右转
            's': (0, 0),      # 停止
            '\x1b[A': (1, 0),  # 上箭头 -> 前进
            '\x1b[B': (-1, 0), # 下箭头 -> 后退
            '\x1b[C': (0, -1), # 右箭头 -> 右转
            '\x1b[D': (0, 1),  # 左箭头 -> 左转
        }

    def update_speed_level(self, level):
        self.speed_level = int(level)
        self.get_logger().info(f'速度等级设置为: {self.speed_level}')

    def set_twist(self, linear, angular):
        # 根据速度等级计算线速度和角速度
        speed_factor = 0.5 * self.speed_level
        self.current_twist.linear.x = linear * speed_factor * self.max_speed
        self.current_twist.angular.z = angular * speed_factor * self.max_speed
        self.get_logger().info(f'线速度: {self.current_twist.linear.x}, 角速度: {self.current_twist.angular.z}')

    def publish_twist(self):
        self.publisher_.publish(self.current_twist)

    def run(self):
        with KeyboardReader() as kb_reader:
            self.get_logger().info('键盘控制已启动，等待输入...')
            try:
                while rclpy.ok():
                    key = kb_reader.get_key()
                    print(f'按键: {key}')

                    if key in ['1', '2', '3']:
                        self.update_speed_level(key)
                        continue

                    if key in self.key_mapping:
                        linear, angular = self.key_mapping[key]
                        self.set_twist(linear, angular)
                    elif key == '\x03':  # 检查Ctrl-C退出
                        break
                    else:
                        self.get_logger().warn('无效按键，请重新输入')

                    self.publish_twist()

            except Exception as e:
                self.get_logger().error(f'错误: {e}')
            finally:
                self.get_logger().info('节点关闭...')
                self.set_twist(0, 0)
                self.publish_twist()

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
