#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <math.h>
#include <chrono>
#include <thread>
#include <deque>
#include <chrono>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <array>
#include "sys/time.h"

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp> 

#include <eigen3/Eigen/Geometry>

#include "mick_chassis_msg.h"
#include "mick_chassis_protocol.h"
 
 
using namespace std;
 
class MickRobotNode : public rclcpp::Node
{
public:
    MickRobotNode()
        :Node("mick_robot_chassis")
    {
        // 参数声明和获取
		declare_parameter<std::string>("sub_cmdvel_topic",sub_cmdvel_topic);
		declare_parameter<std::string>("pub_odom_topic",pub_odom_topic);
		declare_parameter<std::string>("pub_imu_topic",pub_imu_topic);
		declare_parameter<std::string>("joy_topic",joy_topic);

		declare_parameter<std::string>("dev",dev);
		declare_parameter<int>("baud",baud);
		//declare_parameter<int>("time_out",time_out);
		//declare_parameter<int>("hz",hz);
		declare_parameter<double>("delay", delay);

		declare_parameter<int>("is_pub_path",is_pub_path);
		declare_parameter<int>("chassis_type",chassis_type);

		RCLCPP_INFO_STREAM(this->get_logger(),"sub_cmdvel_topic:   "<<sub_cmdvel_topic);
		RCLCPP_INFO_STREAM(this->get_logger(),"pub_odom_topic:   "<<pub_odom_topic);
		RCLCPP_INFO_STREAM(this->get_logger(),"pub_imu_topic:   "<<pub_imu_topic);
		RCLCPP_INFO_STREAM(this->get_logger(),"joy_topic:   " << joy_topic);

		RCLCPP_INFO_STREAM(this->get_logger(),"dev:   "<<dev);
		RCLCPP_INFO_STREAM(this->get_logger(),"baud:   "<<baud);
		RCLCPP_INFO_STREAM(this->get_logger(),"is_pub_path:   "<<is_pub_path);
		RCLCPP_INFO_STREAM(this->get_logger(),"chassis_type:   "<<chassis_type);


		command_sub = this->create_subscription<geometry_msgs::msg::Twist>(sub_cmdvel_topic, 10
												,std::bind(&MickRobotNode::cmd_vel_callback,this,std::placeholders::_1));

		joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic,20);
		imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(pub_imu_topic,rclcpp::QoS(20).transient_local());
		odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic,20);
		path_pub_ = this->create_publisher<nav_msgs::msg::Path>(pub_odom_topic+"/path",rclcpp::QoS(20).transient_local());
        // TF广播器初始化
        odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  
        //declare_parameter<double>("delay", 0.0);

        dev = get_parameter("dev").as_string();
        baud = get_parameter("baud").as_int();
       
		// 初始化底盘串口/参数等
		mick_robot_chassis_ = new MickRobot::MickRobotCHASSIS(dev,baud,chassis_type);
		// 设置定时调用函数，用于读取串口缓冲区
        timer_ = create_wall_timer(10ms, std::bind(&MickRobotNode::read_uart_buffer, this));
    }

private:
	MickRobot::MickRobotCHASSIS* mick_robot_chassis_;
	nav_msgs::msg::Path path;

	std::string sub_cmdvel_topic = "cmd_vel";
	std::string pub_odom_topic = "odom";
	std::string pub_imu_topic = "Imu";
	std::string joy_topic = "rc_remotes/joy";

	int is_pub_path = 0;
	std::string dev = "/dev/ttyUSB0";
	int baud = 115200;
	double delay = 0; 
	int chassis_type = 0;


    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub;
	

	void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
	{
		float speed_x,speed_y,speed_w;
		speed_x = msg->linear.x;
		speed_y = 0;
		speed_w = msg->angular.z;
		 
		mick_robot_chassis_->send_speed_to_chassis(speed_x, speed_y, speed_w);
	}

	void read_uart_buffer(void)
	{
		if(mick_robot_chassis_->readData())
		{
			if(mick_robot_chassis_->chassis_measurements.imu_measurements.available)
			{
				mick_robot_chassis_->chassis_measurements.imu_measurements.available = 0;
				publish_imu(mick_robot_chassis_->chassis_measurements.imu_measurements);
			}
			if(mick_robot_chassis_->chassis_measurements.odom_measurements.available)
			{
				mick_robot_chassis_->chassis_measurements.odom_measurements.available = 0;
				publish_odomtery(mick_robot_chassis_->chassis_measurements.odom_measurements);
			}
			
			 
		}
	}

/**
 * @function 发布里程计的数据
 * 
 */
void publish_odomtery(odom_measure_t& odom_m)
{
 	rclcpp::Clock clock;
	// static tf::TransformBroadcaster odom_broadcaster_;  //定义tf对象
    
    geometry_msgs::msg::TransformStamped odom_trans;  //创建一个tf发布需要使用的TransformStamped类型消息
    geometry_msgs::msg::Quaternion odom_quat;   //四元数变量
    nav_msgs::msg::Odometry odom;  //定义里程计对象
		
	//里程计的偏航角需要转换成四元数才能发布
	odom_quat = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(odom_m.yaw / 2), std::cos(odom_m.yaw / 2)));

	//载入坐标（tf）变换时间戳
	odom_trans.header.stamp = clock.now();
	//发布坐标变换的父子坐标系
	odom_trans.header.frame_id = "odom";     
	odom_trans.child_frame_id = "base_link";       
	//tf位置数据：x,y,z,方向
	odom_trans.transform.translation.x = odom_m.x;
	odom_trans.transform.translation.y = odom_m.y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;        
	//发布tf坐标变化
	odom_broadcaster_->sendTransform(odom_trans);

	//载入里程计时间戳
	odom.header.stamp = clock.now();
	//里程计的父子坐标系
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";       
	//里程计位置数据：x,y,z,方向
	odom.pose.pose.position.x = odom_m.x;     
	odom.pose.pose.position.y = odom_m.y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;       
	//载入线速度和角速度
	odom.twist.twist.linear.x = odom_m.vx;
	odom.twist.twist.linear.y = odom_m.vy;
	odom.twist.twist.angular.z = odom_m.vz;    
	//发布里程计
	odom_pub_->publish(odom);

	if(is_pub_path)
	{
		//发布小车里程计数据推算的轨迹
		geometry_msgs::msg::PoseStamped this_pose_stamped;
		this_pose_stamped.pose = odom.pose.pose;

		path.header.stamp=odom.header.stamp;
		path.header.frame_id="odom";
		path.poses.push_back(this_pose_stamped);
		path_pub_->publish(path);
	}
}
void publish_imu(imu_measure_t& imu_m)
{
	rclcpp::Clock clock;
	sensor_msgs::msg::Imu IMU_msg;  //定义里程计对象
	//载入时间戳
	IMU_msg.header.stamp = clock.now();
	IMU_msg.header.frame_id = "imu";        

	Eigen::Vector3d ea0(imu_m.roll * M_PI / 180.0,
			imu_m.pitch * M_PI / 180.0,
			imu_m.yaw * M_PI / 180.0);
	Eigen::Matrix3d R;
	R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

	Eigen::Quaterniond q;
	q = R;
	q.normalize();

	IMU_msg.orientation.w = (double)q.w();
	IMU_msg.orientation.x = (double)q.x();
	IMU_msg.orientation.y = (double)q.y();
	IMU_msg.orientation.z = (double)q.z();

	IMU_msg.angular_velocity.x = imu_m.gx;   
	IMU_msg.angular_velocity.y = imu_m.gy;   
	IMU_msg.angular_velocity.z = imu_m.gz; 

	IMU_msg.linear_acceleration.x = imu_m.ax;   
	IMU_msg.linear_acceleration.y = imu_m.ay;   
	IMU_msg.linear_acceleration.z = imu_m.az; 

	// double accel = sqrt(imu_m.ax^2 + imu_m.ay^2 +imu_m.az^2 );//归一化
	// IMU_msg.linear_acceleration.x = imu_m.ax/accel*9.8;   
	// IMU_msg.linear_acceleration.y = imu_m.ay/accel*9.8;   
	// IMU_msg.linear_acceleration.z = imu_m.az/accel*9.8; 
 
	//发布IMU
	imu_pub_->publish(IMU_msg);	 
}
// // 发布遥控器数据
// bool publish_joy_msg(rclcpp::Node::SharedPtr node,rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_)
// {
// 	if(mick_chassis.rc.update != 0x01)
// 	{
// 		RCLCPP_WARN_STREAM(this->get_logger(),"rc.update != 0x01 ");
// 		return false;
// 	}

// 	// 0 未标定   1 标定中   2标定完成
// 	if(rc_init_flags != 2)
// 	{
// 		RCLCPP_WARN_STREAM(this->get_logger(),"rc.state != 0x02");
// 		int flag = calculate_rc_offset(node);
// 		//ROS_WARN_STREAM("flag "<<flag<<"  rc.state "<<rc.state);
// 		if(flag == 0)
// 		{
// 			rc_init_flags = 0 ;
// 			RCLCPP_WARN_STREAM(this->get_logger(),"calculate_rc_offset failed .... ");
// 		}
// 		else if(flag == 1)
// 		{
// 			rc_init_flags =1 ;
// 			RCLCPP_WARN_STREAM(this->get_logger(),"initial .... ");
// 		}
// 	}
// 	else
// 	{
// 		if (mick_chassis.rc.available == 1) 
// 		{
			 
// 			float ch1,ch2,ch3,ch4;
			 
// 			float rc_k = 1;

// 			if (mick_chassis.rc.sw2 == 1)            rc_k = 1;
// 			else if (mick_chassis.rc.sw2 == 3)   rc_k = 2;
// 			else if (mick_chassis.rc.sw2 == 2)  	rc_k = 3; // 3m/s
// 			else 				rc_k = 0;
			
// 			// 设置死区
// 			ch1 = (mick_chassis.rc.ch1 - mick_chassis.rc.ch1_offset) / (RC_MAX - mick_chassis.rc.ch1_offset);
// 			if(abs(ch1)<0.2) ch1=0;
// 			ch2 =(mick_chassis.rc.ch2 - mick_chassis.rc.ch2_offset) / (RC_MAX - mick_chassis.rc.ch2_offset);
// 			if(abs(ch2)<0.2) ch2=0;
// 			ch3 = (mick_chassis.rc.ch3 - mick_chassis.rc.ch3_offset) / (RC_MAX - mick_chassis.rc.ch3_offset);
// 			if(abs(ch3)<0.2) ch3=0;
// 			ch4 =(mick_chassis.rc.ch4 - mick_chassis.rc.ch4_offset) / (RC_MAX - mick_chassis.rc.ch4_offset);
// 			if(abs(ch4)<0.2) ch4=0;

// 			sensor_msgs::msg::Joy joy_msg;
// 			joy_msg.axes.push_back(RC_K *rc_k*ch1);
// 			joy_msg.axes.push_back(RC_K *rc_k*ch2);
// 			joy_msg.axes.push_back(RC_K *rc_k*ch3);
// 			joy_msg.axes.push_back(RC_K *rc_k*ch4);
// 			joy_msg.buttons.push_back(mick_chassis.rc.sw1);
// 			joy_msg.buttons.push_back(mick_chassis.rc.sw2);
// 			joy_pub_->publish(joy_msg);
 
// 			return 1;
// 		}
// 	}
// 	mick_chassis.rc.update = 0;
	 
// 	return 1;
// }
// // 计算遥控器的中位值
// int calculate_rc_offset(rclcpp::Node::SharedPtr node)

// {
// 	int re_flag = 0;
// 	if(init_times < 20)
// 	{
// 		if ((mick_chassis.rc.ch1 > 900 && mick_chassis.rc.ch1 < 1100) && (mick_chassis.rc.ch2 > 900 && mick_chassis.rc.ch2 < 1100)
// 			&& (mick_chassis.rc.ch3 > 900 && mick_chassis.rc.ch3 < 1100) && (mick_chassis.rc.ch4 > 900 && mick_chassis.rc.ch4 < 1100))
// 		{
// 			sum_offset[0] += mick_chassis.rc.ch1;
// 			sum_offset[1] += mick_chassis.rc.ch2;
// 			sum_offset[2] += mick_chassis.rc.ch3;
// 			sum_offset[3] += mick_chassis.rc.ch4;
// 			mick_chassis.rc.available = 0;
// 			rc_init_flags = 1; // 0 未标定   1 标定中   2标定完成
// 			init_times++;
// 			RCLCPP_WARN_STREAM(this->get_logger(),"calibrate...");
// 			re_flag = 1;
// 		}
// 	}
// 	else
// 	{
// 		mick_chassis.rc.ch1_offset = sum_offset[0] / init_times;
// 		mick_chassis.rc.ch2_offset = sum_offset[1] / init_times;
// 		mick_chassis.rc.ch3_offset = sum_offset[2] / init_times;
// 		mick_chassis.rc.ch4_offset = sum_offset[3] / init_times;
// 		RCLCPP_INFO_STREAM(this->get_logger(),"ch1_offset: " <<mick_chassis.rc.ch1_offset << " ch2_offset: " << mick_chassis.rc.ch2_offset
// 				 << "ch3_offset: " <<mick_chassis.rc.ch3_offset << " ch4_offset: " << mick_chassis.rc.ch4_offset);
// 		if (mick_chassis.rc.ch1_offset == 0 || mick_chassis.rc.ch2_offset == 0 || mick_chassis.rc.ch3_offset == 0 || mick_chassis.rc.ch4_offset == 0)
// 		{
// 			mick_chassis.rc.available = 0;
// 			rc_init_flags = 0;
// 			init_times = 0;
// 			sum_offset[0] = 0;
// 			sum_offset[1] = 0;
// 			sum_offset[2] = 0;
// 			sum_offset[3] = 0;
// 			RCLCPP_WARN_STREAM(this->get_logger(),"calibrate faild...");
// 			re_flag = 0;
// 		}
// 		else
// 		{
// 			mick_chassis.rc.available = 1;
// 			rc_init_flags = 0x02;
// 			RCLCPP_INFO_STREAM(this->get_logger(),"remote calibrate successful");
// 			re_flag = 2; //标定成功
// 		}
// 	}
// 	return re_flag; //标定中
// }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto chassis_node = std::make_shared<MickRobotNode>();
    rclcpp::spin(chassis_node);
    rclcpp::shutdown();
    return 0;
}

