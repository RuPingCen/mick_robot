
/**
 * 手柄右上亮
 */
 
#include <sys/time.h>
#include <stdbool.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <math.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h> 
#include <std_msgs/String.h>

#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/Twist.h>

#include <gsy4ws4wd_bringup/ultrasonic.h>

#include "chassis_4ws4wd/chassis_4ws4wd.h"  //tcp_client is the directory contained in include directory 




ledParam led_control_para ={0};
chassis_motion_cmd motion_cmd_para={0};


ros::Publisher odom_pub,path_pub,ultra_pub;
geometry_msgs::Twist twist; 
nav_msgs::Path path;

extern chassis_info_check_feedback_ chassis_feedback_info;
//--------------------------
// UDP 參數
extern int udp_sock;
extern struct sockaddr_in servaddr;
extern struct sockaddr_in  src_addr;
extern socklen_t len;
//---------------------------


void run(chassis_motion_cmd &motion_cmd_para);
void param_init(ros::NodeHandle &nh_);
void calculate_position_for_odometry(void);
void publish_odomtery(float  position_x,float position_y,float oriention,
					float vel_linear_x,float vel_linear_y,float vel_linear_w);
void publish_ultra_data(void);
 
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	motion_cmd_para.v = msg->linear.x;
	motion_cmd_para.w = msg->angular.z;
	motion_cmd_para.motion_state = 1;  // 0 带缓冲滑行尽快的停止    1 設置速度    2 急停  
 	write_motion_cmd(motion_cmd_para);
 	//ROS_INFO_STREAM("speed_x:"<<msg->linear.x<<"      speed_y:"<<msg->linear.y<<"      speed_w:"<<msg->angular.z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "chassis_4ws4wd");// ROS节点初始化

	ros::NodeHandle nh_("~"); // 创建节点句柄
	ros::Rate loop_rate(100);  //Hz
 	 
	//ros::Subscriber user_point_sub_;
	ros::Subscriber command_sub = nh_.subscribe("/cmd_vel", 10, cmd_vel_callback);
	odom_pub= nh_.advertise<nav_msgs::Odometry>("/4ws4wd/odom", 20); //定义要发布/odom主题
	path_pub = nh_.advertise<nav_msgs::Path>("/4ws4wd/path",20, true);
    ultra_pub = nh_.advertise<gsy4ws4wd_bringup::ultrasonic>("/4ws4wd/ultrasonic", 10);

	if(chassis_init()<0)//udp initialization
	{
		std::cout << "chassis init filed, now exit ... " << std::endl;
		return -1;
	}


// 	led_control_para.channel = 1 ;
// //	led_control_para.mode =!led_control_para.mode;
// 	led_control_para.mode = 0x00;
// 	led_control_para.on_lightness=1;
// 	encode_led_cmd(led_control_para);
 

	//ros::spin();  
    while(ros::ok())
	{
	 	run(motion_cmd_para);
		ros::spinOnce();                 
		loop_rate.sleep();
	}

	udp_close();
    return 0;
}

/**
  *@ run function : cyclic function in main function   
  * 
  *@ input :   
  *			steering_feedback_data : recevied frame data from chassis
  *			recv_length : length of frame data 
  *@ output : 
  *			  	 	   
**/
void run(chassis_motion_cmd &motion_cmd_para )
{
	short int ret=0;
	short int recv_length=0;
	static int number=1;
	unsigned char steering_feedback_data[MAXLEN] = {0};
	number++;
	recv_length=recvfrom(udp_sock,steering_feedback_data, 1024, 0,(struct sockaddr*)&src_addr, &len);//&number
	if(recv_length>0)
	{
	 	//printf("motion_state : %.2x \t   vel_gear ：%.2x \n",motion_cmd_para.motion_state,motion_cmd_para.vel_gear);
	 	//printf("[%s:%d]",inet_ntoa(src_addr.sin_addr),ntohs(src_addr.sin_port));//打印消息发送方的ip与端口号
	 	//std::cout << "Received " << number << "th data (with length "<< recv_length<<")："<<std::endl;
	 	//
	 	//for(int i=0;i<recv_length;i++)
	 	//{
	 	//	printf("%.2x ",steering_feedback_data[i]);
	 	//}
		//std::cout<<std::endl;
		ret = decode_cmd(steering_feedback_data, recv_length);
	}
	else
	{
		std::cout << "No recevied data !" << std::endl;
	}

	if(chassis_feedback_info.update_flag&0x01) //motion data is updated
	{
		chassis_feedback_info.update_flag &= 0xfe;
		calculate_position_for_odometry();
	}
	if(chassis_feedback_info.update_flag&0x02) //odom data is updated
	{
		chassis_feedback_info.update_flag &= 0xfD;
	}
	if(chassis_feedback_info.update_flag&0x04) //ultra data is updated
	{
		chassis_feedback_info.update_flag &= 0xfB;
		publish_ultra_data();
	}

 	
	if(number%50==0)
	{
		read_odometry_cmd(); // 絕對位移
	}
	if(number%10==0)
	{	
		read_ultrasonic_cmd();	
	}
	if(number%20==0)
	{		
		read_antiColBar_cmd();
	}
	if(number%100==0)
	{
		read_driver_state_cmd(0);	//left-side wheel
		read_driver_state_cmd(1);	//right-side wheel	
	}
	if(number>10000)
	{
		number=1;
	}		
}
 

/**
 * @function 利用里程计数据实现位置估计
 * 
 */
ros::Time last_time;
float position_x=0,position_y=0,position_w=0;
static int motor_init_flag = 0;
void calculate_position_for_odometry(void)
 {
  float linear_x,linear_y,linear_w;
  float distance_delta,theta_delta;
   
   std::cout<<"[\033[32mdecode motion_CmdId\033[0m] : "
				<<"current v(mm/s) : "<< chassis_feedback_info.v 
				<<"\tcurrent W(0.1deg/s) : "<<chassis_feedback_info.w<<std::endl;
std::cout<<"[odom_info] : current mileage(mm) : "<< chassis_feedback_info.mileage
		<<"\tcurrent angle(deg) : "<<chassis_feedback_info.angle<<std::endl;
      
  if(motor_init_flag == 0)
  {
	last_time = ros::Time::now();
	motor_init_flag++;//保证程序只进入一次
	publish_odomtery(0,0,0,0,0,0);
	return ;
  }
  ros::Time current_time = ros::Time::now();
  double dt = current_time.toSec() - last_time.toSec();
  last_time = current_time;
  if(dt>1) dt=1;
   std::cout<<"dt: "<<dt<<std::endl;

   linear_x =  chassis_feedback_info.v/1000.0;
   linear_y = 0;
   linear_w = chassis_feedback_info.w/57.3/10;

  distance_delta =linear_x*dt;
  theta_delta = linear_w*dt;


  position_x = position_x + distance_delta*cos(position_w);
  position_y = position_y + distance_delta*sin(position_w);
  position_w = position_w + theta_delta;

    publish_odomtery( position_x,position_y,position_w,linear_x,linear_y,linear_w);
    //方法２;利用轮子的转速来推算
}
/**
 * @function 发布里程计的数据
 * 
 */
void publish_odomtery(float  position_x,float position_y,float oriention,float vel_linear_x,float vel_linear_y,float vel_linear_w)
{
	static tf::TransformBroadcaster odom_broadcaster;  //定义tf对象
	geometry_msgs::TransformStamped odom_trans;  //创建一个tf发布需要使用的TransformStamped类型消息
	geometry_msgs::Quaternion odom_quat;   //四元数变量
	nav_msgs::Odometry odom;  //定义里程计对象
		
	//里程计的偏航角需要转换成四元数才能发布
	odom_quat = tf::createQuaternionMsgFromYaw(oriention);//将偏航角转换成四元数

	//载入坐标（tf）变换时间戳
	odom_trans.header.stamp = ros::Time::now();
	//发布坐标变换的父子坐标系
	odom_trans.header.frame_id = "odom";     
	odom_trans.child_frame_id = "base_link";       
	//tf位置数据：x,y,z,方向
	odom_trans.transform.translation.x = position_x;
	odom_trans.transform.translation.y = position_y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;        
	//发布tf坐标变化
	odom_broadcaster.sendTransform(odom_trans);

	//载入里程计时间戳
	odom.header.stamp = ros::Time::now(); 
	//里程计的父子坐标系
	odom.header.frame_id = "odom";
	odom.child_frame_id = "base_link";       
	//里程计位置数据：x,y,z,方向
	odom.pose.pose.position.x = position_x;     
	odom.pose.pose.position.y = position_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;       
	//载入线速度和角速度
	odom.twist.twist.linear.x = vel_linear_x;
	odom.twist.twist.linear.y = vel_linear_y;
	odom.twist.twist.angular.z = vel_linear_w;    
	//发布里程计
	odom_pub.publish(odom);

	//发布小车里程计数据推算的轨迹
	geometry_msgs::PoseStamped this_pose_stamped;
	this_pose_stamped.pose = odom.pose.pose;

	path.header.stamp=odom.header.stamp;
    path.header.frame_id="odom";
	path.poses.push_back(this_pose_stamped);
    path_pub.publish(path);
}
void publish_ultra_data(void)
{
	gsy4ws4wd_bringup::ultrasonic ultra_msg;
	ultra_msg.header.stamp = ros::Time::now();
	ultra_msg.header.frame_id = "base_link";
 	
	for(uint8_t index_i=0;index_i<8;index_i++)
	{
	    ultra_msg.data.push_back(chassis_feedback_info.ultrasonic_distance[index_i]);
	}
	ultra_pub.publish(ultra_msg);
 
}
 
