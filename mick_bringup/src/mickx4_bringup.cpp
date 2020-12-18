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

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include<tf/transform_broadcaster.h>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>

#include <serial/serial.h>
#include <std_msgs/String.h>

#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <sys/time.h>

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

float WHEEL_RATIO =19.0; 		//减速比 3508电机减速比为1:19
float WHEEL_L=0.680;                 //左右轮子的间距
float WHEEL_D=0.254; 		    	//轮子直径  10寸的轮子
float WHEEL_R=WHEEL_D/2.0; 			//轮子半径
float WHEEL_PI=3.141693; 			//pi

struct timeval time_val; //time varible
struct timezone tz;
double time_stamp;
serial::Serial ros_ser;
ros::Publisher odom_pub;

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
		uint16_t 	ax,ay,az;		 
 		uint16_t 	gx,gy,gz;	
 		uint16_t 	mx,my,mz;	
		float pitch,roll,yaw;
		float pitch_rad,roll_rad,yaw_rad;
		
}imu_measure_t;


moto_measure_t moto_chassis[4] = {0};
imu_measure_t imu_chassis;  //IMU 数据
//uint16_t Ultrasonic_data [10];   //超声波数据
vector<uint16_t> Ultrasonic_data(10,0);

union floatData //union的作用为实现char数组和float之间的转换
{
    int32_t int32_dat;
    unsigned char byte_data[4];
}motor_upload_counter,total_angle,round_cnt;
union IntData //union的作用为实现char数组和int16数据类型之间的转换
{
    int16_t int16_dat;
    unsigned char byte_data[2];
}speed_rpm,imu;

 
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
void send_speed_to_chassis(float x,float y,float w);
void send_rpm_to_chassis( int w1, int w2, int w3, int w4);
void clear_odometry_chassis(void);
bool analy_uart_recive_data( std_msgs::String serial_data);
void calculate_position_for_odometry(void);
void publish_odomtery(float  position_x,float position_y,float oriention,float vel_linear_x,float vel_linear_y,float vel_linear_w);

int main(int argc,char** argv)
{
    string out_result;
    bool uart_recive_flag;
 
//     unsigned char buf[200];                      //定义字符串长度
//     boost::asio::io_service iosev;
//     serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
//     sp.set_option(serial_port::baud_rate(115200));
//     sp.set_option(serial_port::flow_control());
//     sp.set_option(serial_port::parity());
//     sp.set_option(serial_port::stop_bits());
//     sp.set_option(serial_port::character_size(8));
 
    string sub_cmdvel_topic,pub_odom_topic,dev;
	int buad,time_out,hz;
 	ros::init(argc, argv, "mickx4");
	 ros::NodeHandle n("~");
	 
	n.param<std::string>("sub_cmdvel_topic", sub_cmdvel_topic, "/cmd_vel");
	n.param<std::string>("pub_odom_topic", pub_odom_topic, "/odom");
	n.param<std::string>("dev", dev, "/dev/mick");
	n.param<int>("buad", buad, 115200);
	n.param<int>("time_out", time_out, 1000);
	n.param<int>("hz", hz, 100);
	
	ROS_INFO_STREAM("sub_cmdvel_topic:   "<<sub_cmdvel_topic);
	ROS_INFO_STREAM("pub_odom_topic:   "<<pub_odom_topic);
	ROS_INFO_STREAM("dev:   "<<dev);
	ROS_INFO_STREAM("buad:   "<<buad);
	ROS_INFO_STREAM("time_out:   "<<time_out);
	ROS_INFO_STREAM("hz:   "<<hz);
	 
	
	 //订阅主题command
	 ros::Subscriber command_sub = n.subscribe(sub_cmdvel_topic, 10, cmd_vel_callback);
	 //发布主题sensor
	   // ros::Publisher sensor_pub = n.advertise<std_msgs::String>("sensor", 1000);
        odom_pub= n.advertise<nav_msgs::Odometry>(pub_odom_topic, 20); //定义要发布/odom主题
	// 开启串口模块
	 try
	 {
			 ros_ser.setPort(dev);
			 ros_ser.setBaudrate(buad);
			 serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			 //serial::Timeout to = serial::Timeout(1,time_out,0,time_out,0);
			 to.inter_byte_timeout=1;
			 to.read_timeout_constant=5;
			 to.read_timeout_multiplier=0;
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
 
 clear_odometry_chassis();
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
		}
		sleep(1);
}
 ROS_INFO_STREAM("clear odometry successful !");
   
    while(ros::ok())
    { 
  
	 if(ros_ser.available() )
	 {
	    //ROS_INFO_STREAM("Reading from serial port");
	    std_msgs::String serial_data;
	    //获取串口数据
	    serial_data.data = ros_ser.read(ros_ser.available());
	   uart_recive_flag = analy_uart_recive_data(serial_data);
	   if(uart_recive_flag)
	   {
            uart_recive_flag=0;
	    calculate_position_for_odometry();
	    //odom_pub.publish(serial_data);//将串口数据发布到主题sensor
	  }
	  // else
	   //{
	      //serial_data.data = ros_ser.read(ros_ser.available());
	    //ros_ser.flushInput(); //清空缓冲区数据
	    //sleep(0.5);            //延时0.1秒,确保有数据进入
	  // }
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
//ROS_INFO_STREAM("Write to serial port" << msg->data);
 // ostringstream os;
  float speed_x,speed_y,speed_w;
  float v1=0,v2=0,v3=0,v4=0;
  // os<<"speed_x:"<<msg->linear.x<<"      speed_y:"<<msg->linear.y<<"      speed_w:"<<msg->angular.z<<'\n';
 //cout<<os.str()<<endl;
//send_speed_to_chassis(msg->linear.x*10,msg->linear.y*10,msg->angular.z*2);

  
  speed_x = msg->linear.x;
  speed_y = 0;
  speed_w = msg->angular.z;
  
//   v1 =speed_x-speed_y-WHEEL_K*speed_w;       //转化为每个轮子的线速度
//   v2 =speed_x+speed_y-WHEEL_K*speed_w;
//   v3 =-(speed_x-speed_y+WHEEL_K*speed_w);
//   v4 =-(speed_x+speed_y+WHEEL_K*speed_w);

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
  ROS_INFO_STREAM("v1: "<<v1<<"      v2: "<<v2<<"      v3: "<<v3<<"      v4: "<<v4);
  ROS_INFO_STREAM("speed_x:"<<msg->linear.x<<"      speed_y:"<<msg->linear.y<<"      speed_w:"<<msg->angular.z);
}

void send_speed_to_chassis(float x,float y,float w)
{
  uint8_t data_tem[50];
  unsigned int speed_0ffset=10; //速度偏移值 10ｍ/s，把速度转换成正数发送
  unsigned char i,counter=0;
  unsigned char  cmd,length;
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
  unsigned int speed_0ffset=10000; //转速偏移１００００转

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
  unsigned int speed_0ffset=10000; //转速偏移１００００转
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
bool  analy_uart_recive_data( std_msgs::String serial_data)
{
  unsigned char reviced_tem[500];
  uint16_t len=0,i=0,j=0;
  unsigned char check=0;
  unsigned char tem_last=0,tem_curr=0,rec_flag=0;//定义接收标志位
  uint16_t header_count=0,step=0; //计数这个数据序列中有多少个帧头
  len=serial_data.data.size();
  if(len<1 || len>500)
  {
	ROS_INFO_STREAM("serial data is too short ,  len: " << serial_data.data.size() );
	std_msgs::String serial_data;
	string str_tem;

	serial_data.data = ros_ser.read(ros_ser.available());
	str_tem =  serial_data.data;
     return false; //数据长度太短　
   }
   //ROS_INFO_STREAM("Read: " << serial_data.data.size() );

   // 有可能帧头不在第一个数组位置
  for( i=0;i<len;i++) 
  {
	 tem_last=  tem_curr;
	 tem_curr = serial_data.data.at(i);
	 if(tem_last == 0xAE && tem_curr==0xEA&&rec_flag==0) //在接受的数据串中找到帧头　
	 {
		 rec_flag=1;
		 reviced_tem[j++]=tem_last;
		 reviced_tem[j++]=tem_curr;
		 //ROS_INFO_STREAM("found frame head" ); 
	}
	else if (rec_flag==1)
	{
		reviced_tem[j++]=serial_data.data.at(i);
		if(tem_last == 0xEF && tem_curr==0xFE)
		{
			header_count++;
			rec_flag=2;
		}
	}
	else
		rec_flag=0;
  }
  // 检验数据长度和校验码是否正确
//   if(reviced_tem[len-3] ==check || reviced_tem[len-3]==0xff)
//     ;
//   else
//     return;
  // 检验接受数据的长度
  step=0;
  for(i=0;i<header_count;i++) 
  {
	  len = (reviced_tem[2+step] +4 ) ; //第一个帧头的长度
	  //cout<<"read head :" <<i<< "      len:   "<<len;
	   if(reviced_tem[0+step] ==0xAE && reviced_tem[1+step] == 0xEA && reviced_tem[len-2+step]==0xEF &&reviced_tem[len-1+step]==0xFE) 
      {//检查帧头帧尾是否完整
		  if (reviced_tem[3+step] ==0x01 )
		  {
			  //ROS_INFO_STREAM("recived motor  data" ); 
				i=4;
				motor_upload_counter.byte_data[3]=reviced_tem[i++];
				motor_upload_counter.byte_data[2]=reviced_tem[i++];
				motor_upload_counter.byte_data[1]=reviced_tem[i++];
				motor_upload_counter.byte_data[0]=reviced_tem[i++];
				for(int j=0;j<4;j++)
				{
					speed_rpm.int16_dat=0;
					total_angle.int32_dat =0;
					round_cnt.int32_dat=0;
					
					speed_rpm.byte_data[1] = reviced_tem[i++] ; 
					speed_rpm.byte_data[0] = reviced_tem[i++] ;

					total_angle.byte_data[3]=reviced_tem[i++]; 
					total_angle.byte_data[2]=reviced_tem[i++];
					total_angle.byte_data[1]=reviced_tem[i++];
					total_angle.byte_data[0]=reviced_tem[i++];
					
					round_cnt.byte_data[3]=reviced_tem[i++]; 
					round_cnt.byte_data[2]=reviced_tem[i++];
					round_cnt.byte_data[1] = reviced_tem[i++] ; 
					round_cnt.byte_data[0] = reviced_tem[i++] ;

					moto_chassis[j].angle = reviced_tem[i++] *256; 
					moto_chassis[j].angle += reviced_tem[i++];

					moto_chassis[j].Temp = reviced_tem[i++]; 

					moto_chassis[j].round_cnt =  round_cnt.int32_dat;
					moto_chassis[j].speed_rpm = speed_rpm.int16_dat;
					moto_chassis[j].total_angle = total_angle.int32_dat;
					
					moto_chassis[j].counter = motor_upload_counter.int32_dat;
				}
				// 根据电机安装的位置，第３号和第４号电机方向相反
				moto_chassis[2].speed_rpm = -moto_chassis[2].speed_rpm ;
				moto_chassis[2].total_angle = -moto_chassis[2].total_angle;
				moto_chassis[2].round_cnt = -moto_chassis[2].round_cnt;

				moto_chassis[3].speed_rpm = -moto_chassis[3].speed_rpm ;
				moto_chassis[3].total_angle = -moto_chassis[3].total_angle;
				moto_chassis[3].round_cnt = -moto_chassis[3].round_cnt;
			   ROS_INFO_STREAM("recived motor data" ); 
			   	for(i=0;i<4;i++)
				{
					// 打印四个电机的转速、转角、温度等信息
					 ROS_INFO_STREAM("M "<< i <<": " <<motor_upload_counter.int32_dat<<" ｖ: "<<moto_chassis[i].speed_rpm<<"  t_a: "<<moto_chassis[i].total_angle <<"  n: "<<moto_chassis[i].round_cnt <<"  a: "<<moto_chassis[i].angle );
				   // ROS_INFO_STREAM("M "<< i <<": " <<motor_upload_counter.int32_dat ); 
					//ROS_INFO_STREAM("ｖ : "<<moto_chassis[i].speed_rpm<<"  t_a: "<<moto_chassis[i].total_angle <<"  n: "<<moto_chassis[i].round_cnt <<"  a: "<<moto_chassis[i].angle ); 
					//cout<<"M "<< i <<": " <<motor_upload_counter.int32_dat<<endl;
					//cout<<"ｖ: "<<moto_chassis[i].speed_rpm<<"  t_a: "<<moto_chassis[i].total_angle <<"  n: "<<moto_chassis[i].round_cnt <<"  a: "<<moto_chassis[i].angle<<endl;
				}
		  }
		  else if (reviced_tem[3+step] ==0x10 )
			{
			     i=4+step;
				motor_upload_counter.byte_data[3]=reviced_tem[i++];
				motor_upload_counter.byte_data[2]=reviced_tem[i++];
				motor_upload_counter.byte_data[1]=reviced_tem[i++];
				motor_upload_counter.byte_data[0]=reviced_tem[i++];
			  
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ;imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.ax = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.ay = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.az = imu.int16_dat;
			  
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ;imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.gx = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.gy = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.gz = imu.int16_dat;
			  
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.mx = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.my = imu.int16_dat;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.mz = imu.int16_dat;
			  
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.pitch = imu.int16_dat/100.0f;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.roll = imu.int16_dat/100.0f;
			  imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
			  imu_chassis.yaw = imu.int16_dat/100.0f;
			  
				ROS_INFO_STREAM("recived imu  data" ); 
			}
			else if (reviced_tem[3+step] ==0x11 )
			{
			       i=4+step;
				 uint16_t ultra_tem=0;
				motor_upload_counter.byte_data[3]=reviced_tem[i++];
				motor_upload_counter.byte_data[2]=reviced_tem[i++];
				motor_upload_counter.byte_data[1]=reviced_tem[i++];
				motor_upload_counter.byte_data[0]=reviced_tem[i++];
				for(int j =0;j<10;j++)
				{
				 ultra_tem=reviced_tem[i++]*256; 		ultra_tem = ultra_tem+reviced_tem[i++];
				 Ultrasonic_data.push_back(ultra_tem);
				}
				ROS_INFO_STREAM("recived Ulrat  data" ); 
			}
			else
			{
				ROS_WARN_STREAM("unrecognize frame" ); 
			}
			//return  true;
	  }
	   else
      {
		ROS_WARN_STREAM("frame head is wrong" ); 
         return  false;	
      }
      step+=len; 
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
//  float K4_1 = 1.0/(4.0*WHEEL_K);
  float position_x_delta,position_y_delta,position_w_delta,position_r_delta;
  float linear_x,linear_y,linear_w;

  if((s1_last == 0 && s2_last == 0&& s3_last==0&&s4_last==0) || (moto_chassis[0].counter ==0))
  {
		s1 =    (moto_chassis[0].round_cnt+(moto_chassis[0].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s2 =    (moto_chassis[1].round_cnt+(moto_chassis[1].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s3 =    (moto_chassis[2].round_cnt+(moto_chassis[2].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		s4 =    (moto_chassis[3].round_cnt+(moto_chassis[3].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
		
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
  s1 =    (moto_chassis[0].round_cnt+(moto_chassis[0].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s2 =    (moto_chassis[1].round_cnt+(moto_chassis[1].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s3 =    (moto_chassis[2].round_cnt+(moto_chassis[2].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
  s4 =    (moto_chassis[3].round_cnt+(moto_chassis[3].total_angle%8192)/8192.0)/WHEEL_RATIO*WHEEL_PI*WHEEL_D ; 
 
  s1_delta=s1-s1_last; //每个轮子位移的增量
  s2_delta=s2-s2_last;
  s3_delta=s3-s3_last;
  s4_delta=s4-s4_last;
  
   // ------------------------------------------------------------------------------------------------------------------------------------------------------------------
   if(abs(s1_delta) < 0.001 )  s1_delta=0;
   if(abs(s2_delta) < 0.001 )  s2_delta=0;
    if(abs(s3_delta) < 0.001 )  s3_delta=0;
   if(abs(s4_delta) < 0.001 )  s4_delta=0;
   
  s1_delta = 0.5*s1_delta+0.5*s2_delta;  
  s4_delta = 0.5*s3_delta+0.5*s4_delta; 

//    if(s1_delta || s2_delta || s3_delta || s4_delta)
//   cout<<"s1_delta:  "<<s1_delta<<"   s2_delta: " <<s2_delta<<"   s3_delta: " <<s3_delta<<"   s4_delta: " <<s4_delta<<endl;
 
  position_w_delta =((s4_delta)- (s1_delta))/float(WHEEL_L); //w 单位为弧度
  if(abs(position_w_delta) < 0.0001)
   position_r_delta=0;
  else
  position_r_delta = ((s4_delta)+(s1_delta))/float(2*position_w_delta);
  position_x_delta=position_r_delta*sin(position_w_delta);
  position_y_delta = position_r_delta*(1-cos(position_w_delta));
   // ------------------------------------------------------------------------------------------------------------------------------------------------------------------
 
  position_x=position_x+cos(position_w)*position_x_delta-sin(position_w)*position_y_delta;
  position_y=position_y+sin(position_w)*position_x_delta+cos(position_w)*position_y_delta;
  position_w=position_w+position_w_delta;
  
  if(position_w>2*WHEEL_PI)
  {
     position_w=position_w-2*WHEEL_PI;	
  }
  else if(position_w<-2*WHEEL_PI)
  {
     position_w=position_w+2*WHEEL_PI;
  }
  else;

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
}
 
