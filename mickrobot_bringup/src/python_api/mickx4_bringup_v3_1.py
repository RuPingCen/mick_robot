"""mickx4_bringup_v3_1.py

ROS2 bringup node for the MickRobot X4 chassis.
Mirrors mickx4_bringup_v3.1.cpp.

  - subscribes to geometry_msgs/Twist (cmd_vel)  → chassis
  - publishes sensor_msgs/Imu, nav_msgs/Odometry, nav_msgs/Path
  - publishes tf odom → base_link
  - reads chassis data via MickRobotCHASSIS (mick_chassis_protocol.py)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu, Joy
from tf2_ros import TransformBroadcaster

from .mick_chassis_protocol import MickRobotCHASSIS


# ---------------------------------------------------------------------------
# Quaternion helpers (no scipy / tf_transformations dependency)
# ---------------------------------------------------------------------------
def _yaw_to_quat(yaw: float) -> Quaternion:
    """Yaw-only quaternion, same formula as the C++ code:
        q = (0, 0, sin(yaw/2), cos(yaw/2))
    """
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


def _euler_to_quaternion(roll: float, pitch: float, yaw: float):
    """Same rotation order as the C++ Eigen code:
        R = AngleAxis(yaw, Z) * AngleAxis(pitch, Y) * AngleAxis(roll, X)
    Returns (x, y, z, w).
    """
    cy = math.cos(yaw   / 2.0);  sy = math.sin(yaw   / 2.0)
    cp = math.cos(pitch / 2.0);  sp = math.sin(pitch / 2.0)
    cr = math.cos(roll  / 2.0);  sr = math.sin(roll  / 2.0)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


# ---------------------------------------------------------------------------
# ROS2 node
# ---------------------------------------------------------------------------
class MickRobotNode(Node):
    def __init__(self):
        super().__init__('mick_robot_chassis')

        self.mick_robot_chassis_ = None
        self.path = Path()

        # Default parameter values (same names as the C++ member variables)
        self.sub_cmdvel_topic_ = 'cmd_vel'
        self.pub_odom_topic_   = 'odom'
        self.pub_imu_topic_    = 'Imu'
        self.joy_topic_        = 'rc_remotes/joy'
        self.communication_mode_ = 0
        self.is_pub_path_ = 0
        self.dev_  = '/dev/ttyUSB0'
        self.baud_ = 115200
        self.delay_ = 0.0
        self.chassis_type_ = 0
        self.udp_ip_   = '192.168.1.30'
        self.udp_port_ = 1230

        # Declare parameters
        self.declare_parameter('sub_cmdvel_topic',     self.sub_cmdvel_topic_)
        self.declare_parameter('pub_odom_topic',       self.pub_odom_topic_)
        self.declare_parameter('pub_imu_topic',        self.pub_imu_topic_)
        self.declare_parameter('joy_topic',            self.joy_topic_)
        self.declare_parameter('dev',                  self.dev_)
        self.declare_parameter('baud',                 self.baud_)
        self.declare_parameter('delay',                self.delay_)
        self.declare_parameter('is_pub_path',          self.is_pub_path_)
        self.declare_parameter('chassis_type',         self.chassis_type_)
        self.declare_parameter('communication_mode',  self.communication_mode_)
        self.declare_parameter('udp_ip',               self.udp_ip_)
        self.declare_parameter('udp_port',             self.udp_port_)

        # Read parameters
        self.dev_              = self.get_parameter('dev').get_parameter_value().string_value
        self.baud_             = self.get_parameter('baud').get_parameter_value().integer_value
        self.sub_cmdvel_topic_ = self.get_parameter('sub_cmdvel_topic').get_parameter_value().string_value
        self.pub_odom_topic_   = self.get_parameter('pub_odom_topic').get_parameter_value().string_value
        self.pub_imu_topic_    = self.get_parameter('pub_imu_topic').get_parameter_value().string_value
        self.joy_topic_        = self.get_parameter('joy_topic').get_parameter_value().string_value
        self.is_pub_path_      = self.get_parameter('is_pub_path').get_parameter_value().integer_value
        self.chassis_type_     = self.get_parameter('chassis_type').get_parameter_value().integer_value
        self.communication_mode_ = self.get_parameter('communication_mode').get_parameter_value().integer_value
        self.udp_ip_           = self.get_parameter('udp_ip').get_parameter_value().string_value
        self.udp_port_         = self.get_parameter('udp_port').get_parameter_value().integer_value

        self.get_logger().info(f"sub_cmdvel_topic_:   {self.sub_cmdvel_topic_}")
        self.get_logger().info(f"pub_odom_topic_:   {self.pub_odom_topic_}")
        self.get_logger().info(f"pub_imu_topic_:   {self.pub_imu_topic_}")
        self.get_logger().info(f"joy_topic_:   {self.joy_topic_}")
        self.get_logger().info(f"dev_:   {self.dev_}")
        self.get_logger().info(f"baud_:   {self.baud_}")
        self.get_logger().info(f"is_pub_path_:   {self.is_pub_path_}")
        self.get_logger().info(f"chassis_type_:   {self.chassis_type_}")
        self.get_logger().info(f"communication_mode_:   {self.communication_mode_}")
        self.get_logger().info(f"udp_ip_:   {self.udp_ip_}")
        self.get_logger().info(f"udp_port_:   {self.udp_port_}")

        # Publishers
        qos_transient = QoSProfile(
            depth=20,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.joy_pub_  = self.create_publisher(Joy,      self.joy_topic_,       20)
        self.imu_pub_  = self.create_publisher(Imu,      self.pub_imu_topic_,   qos_transient)
        self.odom_pub_ = self.create_publisher(Odometry, self.pub_odom_topic_,  20)
        self.path_pub_ = self.create_publisher(Path,     self.pub_odom_topic_ + "/path", qos_transient)

        # Subscriber
        self.command_sub_ = self.create_subscription(
            Twist, self.sub_cmdvel_topic_, self.cmd_vel_callback, 10)

        # TF broadcaster
        self.odom_broadcaster_ = TransformBroadcaster(self)

        # Chassis protocol object
        self.mick_robot_chassis_ = MickRobotCHASSIS(
            self.communication_mode_, self.chassis_type_,
            self.dev_, self.baud_, self.udp_ip_, self.udp_port_)

        # 100 Hz timer for reading from chassis
        self.timer_ = self.create_timer(0.01, self.read_data_buffer)

    # -- callbacks ----------------------------------------------------------
    def cmd_vel_callback(self, msg: Twist):
        if self.mick_robot_chassis_ is None:
            self.get_logger().error('mick_robot_chassis_ is None')
            return
        speed_x = msg.linear.x
        speed_y = 0.0
        speed_w = msg.angular.z
        self.mick_robot_chassis_.send_speed_to_chassis(speed_x, speed_y, speed_w)

    def read_data_buffer(self):
        if self.mick_robot_chassis_ is None:
            self.get_logger().error('mick_robot_chassis_ is None')
            return
        if self.mick_robot_chassis_.readData():
            imu_m = self.mick_robot_chassis_.chassis_measurements.imu_measurements
            if imu_m.available:
                imu_m.available = 0
                self.publish_imu(imu_m)
            odom_m = self.mick_robot_chassis_.chassis_measurements.odom_measurements
            if odom_m.available:
                odom_m.available = 0
                self.publish_odomtery(odom_m)

    # -- publishers ---------------------------------------------------------
    def publish_odomtery(self, odom_m):
        now = self.get_clock().now().to_msg()

        odom_quat = _yaw_to_quat(odom_m.yaw)

        # TF: odom -> base_link
        odom_trans = TransformStamped()
        odom_trans.header.stamp    = now
        odom_trans.header.frame_id = 'odom'
        odom_trans.child_frame_id  = 'base_link'
        odom_trans.transform.translation.x = odom_m.x
        odom_trans.transform.translation.y = odom_m.y
        odom_trans.transform.translation.z = 0.0
        odom_trans.transform.rotation = odom_quat
        self.odom_broadcaster_.sendTransform(odom_trans)

        # Odometry message
        odom = Odometry()
        odom.header.stamp    = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = odom_m.x
        odom.pose.pose.position.y = odom_m.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        odom.twist.twist.linear.x  = odom_m.vx
        odom.twist.twist.linear.y  = odom_m.vy
        odom.twist.twist.angular.z = odom_m.vz
        self.odom_pub_.publish(odom)

        if self.is_pub_path_:
            pose_stamped = PoseStamped()
            pose_stamped.pose = odom.pose.pose
            self.path.header.stamp    = odom.header.stamp
            self.path.header.frame_id = 'odom'
            self.path.poses.append(pose_stamped)
            self.path_pub_.publish(self.path)

    def publish_imu(self, imu_m):
        imu_msg = Imu()
        imu_msg.header.stamp    = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu'

        # Euler (degrees) -> quaternion
        roll  = math.radians(imu_m.roll)
        pitch = math.radians(imu_m.pitch)
        yaw   = math.radians(imu_m.yaw)
        x, y, z, w = _euler_to_quaternion(roll, pitch, yaw)
        imu_msg.orientation.w = w
        imu_msg.orientation.x = x
        imu_msg.orientation.y = y
        imu_msg.orientation.z = z

        imu_msg.angular_velocity.x = imu_m.gx
        imu_msg.angular_velocity.y = imu_m.gy
        imu_msg.angular_velocity.z = imu_m.gz

        imu_msg.linear_acceleration.x = imu_m.ax
        imu_msg.linear_acceleration.y = imu_m.ay
        imu_msg.linear_acceleration.z = imu_m.az

        self.imu_pub_.publish(imu_msg)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    chassis_node = MickRobotNode()
    try:
        rclpy.spin(chassis_node)
    except KeyboardInterrupt:
        pass
    finally:
        chassis_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
