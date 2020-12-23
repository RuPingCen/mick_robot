/**
 * mick四轮差速底盘， 接受cmd_vel 话题的数据，将其转化成转速指令 然后下发到底盘的STM32控制器中
 *  注意：该四轮差速模型与两轮差速模型相同，发送数据的时候需要把1/2号电机的速度设置为一样
 * 3/4号电机速度设置为一样，
 * 
 * 
 * 增加IMU数据上传
 * 增加超声波上传
 * 
 * maker:crp
 * 2017-5-13
 */

#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <math.h>
#include<unistd.h>
#include <ros/ros.h>
#include <ros/spinner.h>
 
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>

#include <tf/tf.h>
#include<tf/transform_broadcaster.h>

#include <serial/serial.h>
#include <std_msgs/String.h>

#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <sstream>

#include <mutex>
#include <thread>

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

float WHEEL_RATIO =19.0; 		//减速比 3508电机减速比为1:19
float WHEEL_L=0.680;                 //左右轮子的间距
float WHEEL_D=0.254; 		    	//轮子直径  10寸的轮子
float WHEEL_R=WHEEL_D/2.0; 			//轮子半径
float WHEEL_PI=3.141693; 			//pi

bool use_imu_topic = false;
float position_w_offset = 0;
typedef struct{
		uint16_t 	angle;				//abs angle range:[0,8191] 电机转角绝对值
		uint16_t 	last_angle;	  //abs angle range:[0,8191]
	
		int16_t	 	speed_rpm;       //转速
 
		int16_t  	given_current;   //实际的转矩电流
		uint8_t  	Temp;           //温度

		uint16_t	offset_angle;   //电机启动时候的零偏角度
		int32_t		round_cnt;     //电机转动圈数
		int32_t		total_angle;    //电机转动的总角度
		
		uint32_t counter;
}moto_measure_t;
typedef struct{
	uint32_t counter;
	float 	ax,ay,az;		 
	float 	gx,gy,gz;	
	float 	mx,my,mz;	
	double pitch,roll,yaw;
	double pitch_rad,roll_rad,yaw_rad;
	float temp;
	float qw,qx,qy,qz;
	
	uint8_t IMUFlag;
	uint8_t GPSFlag;	

	double press,high;
	double GPSLon,GPSLat;
	double GPSHeight,GPSYaw,GPSV;
	double GPSSN,GPSPDOP,GPSHDOP,GPSVDOP;
		
}imu_measure_t;
 
  

struct timeval time_val; //time varible
struct timezone tz;
double time_stamp;

moto_measure_t moto_chassis[4] = {0};
imu_measure_t imu;  //IMU 数据
std::mutex imuMutex;
vector<uint16_t> Ultrasonic_data(12,0);

 
serial::Serial ros_ser;
ros::Publisher odom_pub, imu_pub, mag_pub;
ros::Subscriber sub_imu; 
 
 
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
void sub_imu_callback(const sensor_msgs::Imu::ConstPtr imu_msg);
void send_speed_to_chassis(float x,float y,float w);
void send_rpm_to_chassis( int w1, int w2, int w3, int w4);
void clear_odometry_chassis(void);
bool analy_uart_recive_data(std_msgs::String& serial_data);
void calculate_position_for_odometry(void);
void publish_odomtery(float  position_x,float position_y,float oriention,float vel_linear_x,float vel_linear_y,float vel_linear_w);
void publish_imu_mag(void);

int main(int argc,char** argv)
{
    string out_result;
    bool uart_recive_flag;

    string sub_cmdvel_topic,pub_odom_topic,dev;
	string sub_imu_topic;
	int buad,time_out,hz;

 	ros::init(argc, argv, "mickx4");
	ros::NodeHandle n("~");

	n.param<std::string>("sub_imu_topic", sub_imu_topic, "/imu"); 
	n.param<std::string>("sub_cmdvel_topic", sub_cmdvel_topic, "/cmd_vel");
	n.param<std::string>("pub_odom_topic", pub_odom_topic, "/odom");
	n.param<std::string>("dev", dev, "/dev/mick");
	n.param<int>("buad", buad, 115200);
	n.param<int>("time_out", time_out, 1000);
	n.param<int>("hz", hz, 100);
	n.param<bool>("use_imu_topic", use_imu_topic, false);
	
	ROS_INFO_STREAM("sub_imu_topic:   "<<sub_imu_topic);
	ROS_INFO_STREAM("sub_cmdvel_topic:   "<<sub_cmdvel_topic);
	ROS_INFO_STREAM("pub_odom_topic:   "<<pub_odom_topic);
	ROS_INFO_STREAM("dev:   "<<dev);
	ROS_INFO_STREAM("buad:   "<<buad);
	ROS_INFO_STREAM("time_out:   "<<time_out);
	ROS_INFO_STREAM("hz:   "<<hz);
	ROS_INFO_STREAM("use_imu_topic:   "<<use_imu_topic);  

	//订阅主题command
	ros::Subscriber command_sub = n.subscribe(sub_cmdvel_topic, 10, cmd_vel_callback);
	odom_pub= n.advertise<nav_msgs::Odometry>(pub_odom_topic, 20); //定义要发布/odom主题
	imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 1);
	mag_pub = n.advertise<sensor_msgs::MagneticField>("/mag", 1);
	sub_imu = n.subscribe(sub_imu_topic, 10, sub_imu_callback);
	 
	// 开启串口模块
	 try
	 {
		ros_ser.setPort(dev);
		ros_ser.setBaudrate(buad);
		 
		//serial::Timeout to = serial::Timeout(1,time_out,0,time_out,0);
		// to.inter_byte_timeout=1;
		// to.read_timeout_constant=5;
		// to.read_timeout_multiplier=0;
		serial::Timeout to = serial::Timeout::simpleTimeout(time_out);
		ros_ser.setTimeout(to);
		ros_ser.open();
		ros_ser.flushInput(); //清空缓冲区数据
	 }
	 catch (serial::IOException& e)
	 {
	      ROS_ERROR_STREAM("Unable to open port ");
	      return -1;
	}
	if(ros_ser.isOpen())
	{
	    ros_ser.flushInput(); //清空缓冲区数据
	    ROS_INFO_STREAM("Serial Port opened");
	}
	else
	{
	    return -1;
	}
 
	ros::Rate loop_rate(hz);
 
	bool init_OK=false;
	while(!init_OK)	
	{
		clear_odometry_chassis();
		ROS_INFO_STREAM("clear odometry ..... ");
		if(ros_ser.available())
		{
			std_msgs::String serial_data;
			string str_tem;
			//获取串口数据
			serial_data.data = ros_ser.read(ros_ser.available());
			str_tem =  serial_data.data;
			// cout<<"Recived "<<serial_data.data.c_str()<<endl;
			// ROS_INFO_STREAM(serial_data.data.c_str());
			if(str_tem.find("OK",0) )
				init_OK =true;
			else
				ros_ser.flushInput(); //清空缓冲区数据
		}
		sleep(1);
	}
	ROS_INFO_STREAM("clear odometry successful !");
    //ros_ser.flushInput(); //清空缓冲区数据
    while(ros::ok())
    {  
		if(ros_ser.available())
		{
			std_msgs::String serial_data;
			serial_data.data = ros_ser.read(ros_ser.available());
 
			uart_recive_flag = analy_uart_recive_data(serial_data);
			
			if(uart_recive_flag)
			{
				uart_recive_flag=0;
				calculate_position_for_odometry();
			}
			else
			{
				ROS_WARN_STREAM("analysis data is error ..." ); 
			}
		}	 
		ros::spinOnce();
		loop_rate.sleep();
		
    }
   
    std::cout<<" EXIT ..."<<std::endl;
    ros::waitForShutdown();
    ros::shutdown();

    return 1;

}
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
	float speed_x,speed_w;
	float v1=0,v2=0,v3=0,v4=0;

	speed_x = msg->linear.x;
	speed_w = msg->angular.z;

	v1 =speed_x-0.5*WHEEL_L*speed_w;   //左边    //转化为每个轮子的线速度
	v2 =v1;
	v4 =-(speed_x+0.5*WHEEL_L*speed_w);
	v3 =v4;

	v1 =v1/(2.0*WHEEL_R*WHEEL_PI);    //转换为轮子的速度　RPM
	v2 =v2/(2.0*WHEEL_R*WHEEL_PI);
	v3 =v3/(2.0*WHEEL_R*WHEEL_PI);
	v4 =v4/(2.0*WHEEL_R*WHEEL_PI);

	v1 =v1*WHEEL_RATIO*60;    //转每秒转换到RPM
	v2 =v2*WHEEL_RATIO*60;
	v3 =v3*WHEEL_RATIO*60;
	v4 =v4*WHEEL_RATIO*60;

	send_rpm_to_chassis(v1,v2,v3,v4);	 
	//send_rpm_to_chassis(200,200,200,200);	
	ROS_INFO_STREAM("v1: "<<v1<<"      v2: "<<v2<<"      v3: "<<v3<<"      v4: "<<v4 << 
			"  speed_x:"<<msg->linear.x<<"      speed_w:"<<msg->angular.z);
}
void sub_imu_callback(const sensor_msgs::Imu::ConstPtr imu_msg)
{
	 
	unique_lock<mutex> lck(imuMutex);
    use_imu_topic = true;

	imu.gx = imu_msg->angular_velocity.x;
	imu.gy = imu_msg->angular_velocity.y;
	imu.gz = imu_msg->angular_velocity.z;

	imu.ax = imu_msg->linear_acceleration.x;
	imu.ay = imu_msg->linear_acceleration.y;
	imu.az = imu_msg->linear_acceleration.z;

	imu.qw = imu_msg->orientation.w;
	imu.qx = imu_msg->orientation.x;
	imu.qy = imu_msg->orientation.y;
	imu.qz = imu_msg->orientation.z;

	tf::Quaternion quat(imu.qw,imu.qx,imu.qy,imu.qz);
	tf::Matrix3x3(quat).getRPY(imu.roll_rad,imu.pitch_rad,imu.yaw_rad);
	 
	imu.IMUFlag =0x01;
 
	//ROS_INFO_STREAM("roll: "<<imu.roll_rad*57.3<<"  pitch: "<<imu.pitch_rad*57.3<<"  yaw: "<<imu.yaw_rad*57.3);
	 
}
void send_speed_to_chassis(float x,float y,float w)
{
	uint8_t data_tem[50];
	unsigned int speed_0ffset=10; //速度偏移值 10ｍ/s，把速度转换成正数发送
	unsigned char i,counter=0;
	unsigned char  cmd;
	unsigned int check=0;
	cmd =0xF3; //针对MickX4的小车使用F3 字段      针对MickM4的小车使用F2
	data_tem[counter++] =0xAE;
	data_tem[counter++] =0xEA;
	data_tem[counter++] =0x0B;
	data_tem[counter++] =cmd;

	data_tem[counter++] =((x+speed_0ffset)*100)/256; // X
	data_tem[counter++] =((x+speed_0ffset)*100);

	data_tem[counter++] =((y+speed_0ffset)*100)/256; // X
	data_tem[counter++] =((y+speed_0ffset)*100);

	data_tem[counter++] =((w+speed_0ffset)*100)/256; // X
	data_tem[counter++] =((w+speed_0ffset)*100);

	data_tem[counter++] =0x00;
	data_tem[counter++] =0x00;


	for(i=0;i<counter;i++)
	{
		check+=data_tem[i];
	}
	data_tem[counter++] =0xff;
	data_tem[2] =counter-2;
	data_tem[counter++] =0xEF;
	data_tem[counter++] =0xFE;

	ros_ser.write(data_tem,counter);
}
 
/**
 * @function  发送四个点击的转速到底盘控制器
 * ＠param w1 w2 w3 w4 表示四个电机的转速 单位　ｒｐｍ
 */
void send_rpm_to_chassis( int w1, int w2, int w3, int w4)
{
	uint8_t data_tem[50];
	unsigned int speed_0ffset=10000; //转速偏移10000转

	unsigned char i,counter=0;
	unsigned char  cmd;
	unsigned int check=0;
	cmd =0xF1;
	data_tem[counter++] =0xAE;
	data_tem[counter++] =0xEA;
	data_tem[counter++] =0x0B;
	data_tem[counter++] =cmd;

	data_tem[counter++] =(w1+speed_0ffset)/256; // 
	data_tem[counter++] =(w1+speed_0ffset)%256;

	data_tem[counter++] =(w2+speed_0ffset)/256; // 
	data_tem[counter++] =(w2+speed_0ffset)%256;

	data_tem[counter++] =(w3+speed_0ffset)/256; // 
	data_tem[counter++] =(w3+speed_0ffset)%256;

	data_tem[counter++] =(w4+speed_0ffset)/256; // 
	data_tem[counter++] =(w4+speed_0ffset)%256;

	for(i=0;i<counter;i++)
	{
		check+=data_tem[i];
	}
	data_tem[counter++] =0xff;
	data_tem[2] =counter-2;
	data_tem[counter++] =0xEF;
	data_tem[counter++] =0xFE;

	ros_ser.write(data_tem,counter);
}
 

void clear_odometry_chassis(void)
{
	uint8_t data_tem[50];
	//unsigned int speed_0ffset=10000; //转速偏移1000转
	unsigned char i,counter=0;
	unsigned char  cmd,resave=0x00;
	unsigned int check=0;
	cmd =0xE1;
	data_tem[counter++] =0xAE;
	data_tem[counter++] =0xEA;
	data_tem[counter++] =0x0B;
	data_tem[counter++] =cmd;

	data_tem[counter++] =0x01; //  清零里程计
	data_tem[counter++] =resave;

	data_tem[counter++] =resave; // 
	data_tem[counter++] =resave;

	data_tem[counter++] =resave; // 
	data_tem[counter++] =resave;

	data_tem[counter++] =resave; // 
	data_tem[counter++] =resave;

	for(i=0;i<counter;i++)
	{
		check+=data_tem[i];
	}
	data_tem[counter++] =0xff;
	data_tem[2] =counter-2;
	data_tem[counter++] =0xEF;
	data_tem[counter++] =0xFE;

	ros_ser.write(data_tem,counter);
  
}
 
/**
 * @function 解析串口发送过来的数据帧
 * 成功则返回true　否则返回false
 */
bool analy_uart_recive_data(std_msgs::String& serial_data)
{
	uint8_t reviced_tem[500]; 
	uint16_t data_length=0,i=0,len;
	uint8_t flag=0;
 
	data_length=serial_data.data.size();
	if(data_length<1 || data_length>500)
	{
		//ros_ser.flushInput(); //清空缓冲区数据	
		ROS_WARN_STREAM("serial data is too long ,  len: " << serial_data.data.size() );
		return false;
	}
			  
	for(i=0;i<data_length;i++)
	{	
		reviced_tem[i] =serial_data.data.at(i);
	}

   // 有可能帧头不在第一个数组位置
  for( i=0;i<data_length; ) 
  {
	  
	if(reviced_tem[i] ==0xAE && reviced_tem[1+i] == 0xEA) 
    { 
	  	len = reviced_tem[2+i]+4; //第一个帧头的长度
		 
		uint8_t sum=0x00;
		for(int j=2+i;j<len-3+i;j++)
		{
		    sum+=reviced_tem[j];
		}
		// if(sum != reviced_tem[len-3+i]) 
		// {
		// 	ROS_WARN_STREAM("check sum is error " );  
		// 	cout<<"sum:"<<hex<<sum<<"  "<<reviced_tem[len-3+i];
		// } 	
		if(reviced_tem[len-2+i]==0xEF && reviced_tem[len-1+i]==0xFE) 
      	{   
			if (reviced_tem[3+i] ==0x01 )
			{
				//ROS_INFO_STREAM("recived motor  data" ); 
				uint16_t j=i+4;
				uint32_t counter = (reviced_tem[j] <<24|reviced_tem[j+1] <<16|reviced_tem[j+2] <<8|reviced_tem[j+3]);
				
				for(int n =0;n<4;n++)
				{
					j=i+8+n*13;
					moto_chassis[n].speed_rpm = (reviced_tem[j]<<8|reviced_tem[j+1]);

					int32_t total_angle = (reviced_tem[j+2] <<24|reviced_tem[j+3] <<16|
												reviced_tem[j+4] <<8|reviced_tem[j+5]);


					int32_t round_cnt = (reviced_tem[j+6] <<24|reviced_tem[j+7] <<16|
												reviced_tem[j+8] <<8|reviced_tem[j+9]);
					
					int16_t angle = (reviced_tem[j]<<10|reviced_tem[j+11]);
					int16_t Temp =reviced_tem[j+12];

					moto_chassis[n].total_angle = total_angle;
					moto_chassis[n].round_cnt =  round_cnt;
					moto_chassis[n].angle = angle; 
					moto_chassis[n].Temp = Temp; 
					moto_chassis[n].counter = counter;
				}
				// 根据电机安装的位置，第３号和第４号电机方向相反
				moto_chassis[2].speed_rpm = -moto_chassis[2].speed_rpm ;
				moto_chassis[2].total_angle = -moto_chassis[2].total_angle;
				moto_chassis[2].round_cnt = -moto_chassis[2].round_cnt;

				moto_chassis[3].speed_rpm = -moto_chassis[3].speed_rpm ;
				moto_chassis[3].total_angle = -moto_chassis[3].total_angle;
				moto_chassis[3].round_cnt = -moto_chassis[3].round_cnt;
				// ROS_INFO_STREAM("recived motor data" ); 
				// for(i=0;i<4;i++)
				// {
				// 	//打印四个电机的转速、转角、温度等信息
				// 	ROS_INFO_STREAM("M "<< i <<": " <<moto_chassis[i].counter<<"  V: "<<moto_chassis[i].speed_rpm<<"  t_a: "<<moto_chassis[i].total_angle
				// 	 <<"  n: "<<moto_chassis[i].round_cnt <<"  a: "<<moto_chassis[i].angle );
				// }
				calculate_position_for_odometry();
				flag = 0x01;
			}
			else if (reviced_tem[3+i] ==0x10 )
			{
				uint16_t j =4+i;
				//uint32_t counter = (reviced_tem[j] <<24|reviced_tem[j+1] <<16|reviced_tem[j+2] <<8|reviced_tem[j+3]);
				 
				imu.ax = (reviced_tem[j+4] <<8|reviced_tem[j+5]);
				imu.ay = (reviced_tem[j+6] <<8|reviced_tem[j+7]);
				imu.az = (reviced_tem[j+8] <<8|reviced_tem[j+9]);

				imu.gx = (reviced_tem[j+10] <<8|reviced_tem[j+11]);
				imu.gy = (reviced_tem[j+12] <<8|reviced_tem[j+13]);
				imu.gz = (reviced_tem[j+14] <<8|reviced_tem[j+15]);

				imu.mx = (reviced_tem[j+16] <<8|reviced_tem[j+17]);
				imu.my = (reviced_tem[j+18] <<8|reviced_tem[j+19]);
				imu.mz = (reviced_tem[j+20] <<8|reviced_tem[j+21]);

				 
				imu.pitch = (int16_t)(reviced_tem[j+22] <<8|reviced_tem[j+23])/100.0f;
				imu.roll = (int16_t)(reviced_tem[j+24] <<8|reviced_tem[j+25])/100.0f;
				imu.yaw = (int16_t)(reviced_tem[j+26] <<8|reviced_tem[j+27])/100.0f;

				//ROS_INFO_STREAM("recived imu  data" ); 
				publish_imu_mag();
				flag = 0x01;
			}
			else
			{
				ROS_WARN_STREAM("unrecognize frame" ); 
				flag = 0x00;
			}
		}
	}
	else 
		flag = 0x00;   
	
	if(flag == 0x01)
	{
		i = i+len;
	}
	else
	{
		i = i+1;
	}
   
  }
 return  true;	         
}
/**
 * @function 利用里程计数据实现位置估计
 * 
 */
float s1=0,s2=0,s3=0,s4=0;
float s1_last=0,s2_last=0,s3_last=0,s4_last=0;
float position_x=0,position_y=0,position_w=0;
void calculate_position_for_odometry(void)
 {
	//方法１：　　计算每个轮子转动的位移，然后利用Ｆ矩阵合成Ｘ,Y,W三个方向的位移
	float s1_delta=0,s2_delta=0,s3_delta=0,s4_delta=0;
	float v1=0,v2=0,v3=0,v4=0;
	float position_w_delta;
	float position_delta=0;
	float linear_x,linear_y,linear_w;

	if((s1_last == 0 && s2_last == 0&& s3_last==0&&s4_last==0) || (moto_chassis[0].counter ==0))
	{
		s1 =(moto_chassis[0].round_cnt+(moto_chassis[0].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s2 =(moto_chassis[1].round_cnt+(moto_chassis[1].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s3 =(moto_chassis[2].round_cnt+(moto_chassis[2].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s4 =(moto_chassis[3].round_cnt+(moto_chassis[3].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		
		s1_last=s1;
		s2_last=s2;
		s3_last=s3;
		s4_last=s4;
		
		return ;
	}
	s1_last=s1;
	s2_last=s2;
	s3_last=s3;
	s4_last=s4;
 
	//轮子转动的圈数乘以　N*２*pi*r
	s1 =(moto_chassis[0].round_cnt+(moto_chassis[0].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
	s2 =(moto_chassis[1].round_cnt+(moto_chassis[1].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
	s3 =(moto_chassis[2].round_cnt+(moto_chassis[2].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
	s4 =(moto_chassis[3].round_cnt+(moto_chassis[3].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 

	s1_delta=s1-s1_last; //每个轮子位移的增量
	s2_delta=s2-s2_last;
	s3_delta=s3-s3_last;
	s4_delta=s4-s4_last;

	if(abs(s1_delta) < 0.01 )  s1_delta=0;
	if(abs(s2_delta) < 0.01 )  s2_delta=0;
	if(abs(s3_delta) < 0.01 )  s3_delta=0;
	if(abs(s4_delta) < 0.01 )  s4_delta=0;

	s1_delta = 0.5*s1_delta+0.5*s2_delta;  
	s4_delta = 0.5*s3_delta+0.5*s4_delta; 

	//    if(s1_delta || s2_delta || s3_delta || s4_delta)
	//   cout<<"s1_delta:  "<<s1_delta<<"   s2_delta: " <<s2_delta<<"   s3_delta: " <<s3_delta<<"   s4_delta: " <<s4_delta<<endl;
  
// ------------------------------------------------------------------------------------------------------------------------------------------------------------------

	position_delta = (s4_delta+s1_delta)/2.0;

	position_x=position_x+cos(position_w)*position_delta;
	position_y=position_y+sin(position_w)*position_delta;
  
	if(use_imu_topic == true)//有IMU的时候直接使用IMU输出的航向角
	{
		if(imu.IMUFlag == 0x01) 
		{  
			unique_lock<mutex> lck_imuflag(imuMutex);
			if(position_w_offset == 0)
				position_w_offset = imu.yaw_rad;
			else
				position_w=imu.yaw_rad - position_w_offset;
				
			imu.IMUFlag = 0x00;
		}
	}
	else 
	{
		position_w_delta =((s4_delta)- (s1_delta))/float(WHEEL_L); //w 单位为弧度
		 	
		if(abs(position_w_delta) < 0.017*2)//小于1度
			position_w_delta=0;

		position_w = position_w + position_w_delta;
	}
 
	if(position_w>2*WHEEL_PI)
	{
		position_w=position_w-2*WHEEL_PI;	
	}
	else if(position_w<-2*WHEEL_PI)
	{
		position_w=position_w+2*WHEEL_PI;
	}
	else;

	//方法２;利用轮子的转速来推算
	v1 =    (moto_chassis[0].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2;
	v2 =    (moto_chassis[1].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
	v3 =    (moto_chassis[2].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 
	v4 =    (moto_chassis[3].speed_rpm)/WHEEL_RATIO/60.0*WHEEL_R *WHEEL_PI*2; 

	linear_x = 0.25*v1+ 0.25*v2+ 0.25*v3+ 0.25*v4;
	linear_y = 0;
	linear_w = ((0.5*v3+0.5*v4)-(0.5*v1+0.5*v2))/float(WHEEL_L);
  
  	ROS_INFO_STREAM("px:  "<<position_x<<"   py: " <<position_y<<"   pw: " <<position_w*57.3
  		<<"  vx:  "<<linear_x<<"   vy: " <<linear_y<<"   vw: " <<linear_w<<endl);
 
    publish_odomtery( position_x,position_y,position_w,linear_x,linear_y,linear_w);
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
	//odom_broadcaster.sendTransform(odom_trans);

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
}
 
void publish_imu_mag(void)
{
	static sensor_msgs::Imu imu_msg;
	static sensor_msgs::MagneticField mag_msg;
	tf::TransformBroadcaster imu_broadcaster;

	imu_msg.header.stamp = ros::Time::now();
	imu_msg.header.frame_id = "/imu";

	// Eigen::Vector3d ea0(wit_imu.roll * M_PI / 180.0,
	// 	  wit_imu.pitch * M_PI / 180.0,
	// 	  wit_imu.yaw * M_PI / 180.0);
	// Eigen::Matrix3d R;
	// R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
	// 	* Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
	// 	* Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

	// Eigen::Quaterniond q;
	// q = R;
	// q.normalize();

	// imu_msg.orientation.w = (double)q.w();
	// imu_msg.orientation.x = (double)q.x();
	// imu_msg.orientation.y = (double)q.y();
	// imu_msg.orientation.z = (double)q.z();

	imu_msg.orientation.w = imu.qw;
	imu_msg.orientation.x = imu.qx;
	imu_msg.orientation.y = imu.qy;
	imu_msg.orientation.z = imu.qz;

	// change to rad/s       0.0010652 = 2000/32768/57.3
	imu_msg.angular_velocity.x = imu.gx*0.0010652; 
	imu_msg.angular_velocity.y = imu.gy*0.0010652;
	imu_msg.angular_velocity.z = imu.gz*0.0010652;

	imu_msg.linear_acceleration.x = imu.ax/32768.0f*4;
	imu_msg.linear_acceleration.y = imu.ay/32768.0f*4;
	imu_msg.linear_acceleration.z = imu.az/32768.0f*4;
	imu_pub.publish(imu_msg);

    	//imu_broadcaster.sendTransform(tf::StampedTransform(
        //	tf::Transform(tf::Quaternion(imu.qx,imu.qy, imu.qz,
	//	 	imu.qw),tf::Vector3(0, 0, 0)),ros::Time::now(),"world", "imu"));

	mag_msg.magnetic_field.x = imu.mx;
	mag_msg.magnetic_field.y = imu.my;
	mag_msg.magnetic_field.z = imu.mz;
	mag_msg.header.stamp = imu_msg.header.stamp;
	mag_msg.header.frame_id = imu_msg.header.frame_id;
	mag_pub.publish(mag_msg);
}
