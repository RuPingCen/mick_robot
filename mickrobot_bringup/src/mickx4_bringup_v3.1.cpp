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
#include "mick_chassis_protocol.hpp"

 

//#include "serial/serial.h"
 

using namespace std;

using namespace std::chrono_literals;

 


class MickRobotNode : public rclcpp::Node
{
public:
    MickRobotNode()
        : Node("mick_robot_chassis")
    {
        // 参数声明和获取
		declare_parameter<std::string>("sub_cmdvel_topic",sub_cmdvel_topic);
		declare_parameter<std::string>("pub_odom_topic",pub_odom_topic);
		declare_parameter<std::string>("pub_imu_topic",pub_imu_topic);

		declare_parameter<std::string>("dev",dev_);
		declare_parameter<int>("baud",baud_);
		//declare_parameter<int>("time_out",time_out);
		//declare_parameter<int>("hz",hz);
		declare_parameter<double>("delay", delay_);

		declare_parameter<int>("is_pub_path",is_pub_path);
		declare_parameter<int>("chassis_type",chassis_type);

		declare_parameter<std::string>("joy_topic",joy_topic);
		declare_parameter<float>("RC_K",RC_K);
		declare_parameter<float>("RC_MIN",RC_MIN);
		declare_parameter<float>("RC_MAX",RC_MAX);



		RCLCPP_INFO_STREAM(this->get_logger(),"sub_cmdvel_topic:   "<<sub_cmdvel_topic);
		RCLCPP_INFO_STREAM(this->get_logger(),"pub_odom_topic:   "<<pub_odom_topic);
		RCLCPP_INFO_STREAM(this->get_logger(),"pub_imu_topic:   "<<pub_imu_topic);

		RCLCPP_INFO_STREAM(this->get_logger(),"dev:   "<<dev_);
		RCLCPP_INFO_STREAM(this->get_logger(),"baud:   "<<baud_);
		RCLCPP_INFO_STREAM(this->get_logger(),"is_pub_path:   "<<is_pub_path);
		RCLCPP_INFO_STREAM(this->get_logger(),"chassis_type:   "<<chassis_type);

		RCLCPP_INFO_STREAM(this->get_logger(),"joy_topic:   " << joy_topic);
		RCLCPP_INFO_STREAM(this->get_logger(),"RC_K:   " << RC_K);
		RCLCPP_INFO_STREAM(this->get_logger(),"RC_MIN:   " << RC_MIN);
		RCLCPP_INFO_STREAM(this->get_logger(),"RC_MAX:   " << RC_MAX);

		command_sub = this->create_subscription<geometry_msgs::msg::Twist>(sub_cmdvel_topic, 10
												,std::bind(&MickRobotNode::cmd_vel_callback,this,std::placeholders::_1));

		joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic,20);
		imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(pub_imu_topic,rclcpp::QoS(20).transient_local());
		odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(pub_odom_topic,20);
		path_pub_ = this->create_publisher<nav_msgs::msg::Path>(pub_odom_topic+"/path",rclcpp::QoS(20).transient_local());
        // TF广播器初始化
        odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  
        //declare_parameter<double>("delay", 0.0);

        dev_ = get_parameter("dev").as_string();
        baud_ = get_parameter("baud").as_int();
        delay_ = get_parameter("delay").as_double();

        io_service_ = std::make_shared<boost::asio::io_service>();
        serial_port_ = std::make_shared<boost::asio::serial_port>(*io_service_);

        try
        {
            serial_port_->open(dev_);
        }
        catch (boost::system::system_error &error)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port %s with error %s", dev_.c_str(), error.what());
            return;
        }

        if (!serial_port_->is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", dev_.c_str());
            return;
        }

        // 设置串口选项
        typedef boost::asio::serial_port_base sb;
        sb::baud_rate baud_option(baud_);
        sb::flow_control flow_control(sb::flow_control::none);
        sb::parity parity(sb::parity::none);
        sb::stop_bits stop_bits(sb::stop_bits::one);

        serial_port_->set_option(baud_option);
        serial_port_->set_option(flow_control);
        serial_port_->set_option(parity);
        serial_port_->set_option(stop_bits);
 
		if (!serial_port_->is_open())
		{
			RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
			return;
		}
        timer_ = create_wall_timer(10ms, std::bind(&MickRobotNode::readData, this));
    }

private:
	union INT32Data //union的作用为实现char数组和int32之间的转换
	{
		int32_t int32_dat;
		unsigned char byte_data[4];
	}motor_upload_counter,total_angle,round_cnt,speed_rpm;
	union Int16Data //union的作用为实现char数组和int16数据类型之间的转换
	{
		int16_t int16_dat;
		unsigned char byte_data[2];
	}imu,odom;


	std::string sub_cmdvel_topic = "cmd_vel";
	std::string pub_odom_topic = "odom";
	std::string pub_imu_topic = "Imu";
	std::string joy_topic = "rc_remotes/joy";
	int chassis_type = 0; //默认采用差速模式  0：差速  1-麦克纳姆轮  2: Ackermann  3:4WS4WD
	int is_pub_path = 0; //默认不发布小车底盘轨迹  0：不发布   1 发布

	int rc_init_flags =0;
	unsigned int init_times = 0;
	int sum_offset[4] = {0};
	int show_message =1;
	float RC_MIN = 0, RC_MAX = 2500, RC_K = 1; //遥控器摇杆通道输出的最小值、最大值、比例系数

	//serial::Serial ros_ser;
 


	std::string dev_ = "/dev/ttyUSB0";
	int baud_ = 115200;
    double delay_ =0; 

    std::shared_ptr<boost::asio::io_service> io_service_;
    std::shared_ptr<boost::asio::serial_port> serial_port_;

    rclcpp::TimerBase::SharedPtr timer_;
   
    std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
	rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub;



	chassis_measure_t mick_chassis;
 	nav_msgs::msg::Path path;
	// 计算里程计 用于存储全局的位置
	float position_x=0,position_y=0,position_w=0;
	int motor_init_flag = 0;
	double last_time=0, curr_time =0;

void readData()
{


    // 读取数据
    uint8_t tmp[1024] ;
	std::string tmp_string;
	
    size_t bytes_read = serial_port_->read_some(boost::asio::buffer(tmp, 1024));
    if (bytes_read <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "No data read from serial port");
        return;
    }
 
    // 输出读取的数据以进行调试
    //RCLCPP_INFO(this->get_logger(), "Read %zu bytes", bytes_read);
    for (size_t i = 0; i < bytes_read; ++i)
    {
		tmp_string.push_back(tmp[i]);
        //RCLCPP_INFO(this->get_logger(), "Data[%zu]: 0x%02X", i, tmp_string.at(i));
    }

	if(analy_uart_recive_data(tmp_string))
	{
		if(chassis_type == 0 || chassis_type == 1)
		{
			calculate_chassisDiffX4_position_for_odometry();
		}
		else
		{
			;
		}
	}
	else
	{
		RCLCPP_WARN(this->get_logger()," analy uart recive data error ...");
	}
     
}


void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	float speed_x,speed_y,speed_w;
	speed_x = msg->linear.x;
	speed_y = 0;
	speed_w = msg->angular.z;
	
	int chassis_type = 0; //默认采用差速模式  0：差速  1-麦克纳姆轮  2: Ackermann  3:4WS4WD
	send_speed_to_X4chassis(serial_port_, speed_x,speed_y,speed_w);
}



/**
 * @function 解析串口发送过来的数据帧
 * 成功则返回true　否则返回false
 */
bool analy_uart_recive_data(std::string& str_data)
{
	unsigned char reviced_tem[500];
	uint16_t len=0,i=0,j=0;
	unsigned char check_sum=0;
	unsigned char tem_last=0,tem_curr=0,rec_flag=0;//定义接收标志位
	uint16_t header_count=0,step=0; //计数这个数据序列中有多少个帧头
	len=str_data.size();
	// std::cout<< "=================================================="<<std::endl;
	// for (size_t i = 0; i < len; ++i)
    // {
    //     printf(" 0x%02X  ", str_data[i]);
    // }
	// std::cout<< std::endl;
	if(len<5)
	{
		std::cout<<"serial data is too short ,  len: " << str_data.size()<<std::endl;
		std::cout<< std::hex<<str_data<<std::endl;
		std::cout<< "=================================================="<<std::endl;
		// std_msgs::msg::String serial_data;
		// string str_tem;

		// serial_data.data = ros_ser.read(ros_ser.available());
		// str_tem =  serial_data.data;
		return false; //数据长度太短　
	}
	//ROS_INFO_STREAM("Read: " << serial_data.data.size() );

	// 有可能帧头不在第一个数组位置
	for( i=0;i<len;i++) 
	{
		tem_last=  tem_curr;
		tem_curr = str_data.at(i);
		if(tem_last == 0xAE && tem_curr==0xEA && rec_flag==0) //在接受的数据串中找到帧头　
		{
			rec_flag=1;
			reviced_tem[j++]=tem_last;
			reviced_tem[j++]=tem_curr;
			//ROS_INFO_STREAM("found frame head" ); 
		}
		else if (rec_flag==1)
		{
			reviced_tem[j++]=str_data.at(i);
			if(tem_last == 0xEF && tem_curr==0xFE)
			{
				header_count++;
				rec_flag=0;
			}
		}
		else
			rec_flag=0;
	}
		 

	step=0;
	for(int k=0;k<header_count;k++) 
	{
		len = (reviced_tem[2+step] +4 ) ; //第一个帧头的长度

		//cout<<"read head :" <<i<< "      len:   "<<len;
		if(reviced_tem[0+step] ==0xAE && reviced_tem[1+step] == 0xEA && reviced_tem[len-2+step]==0xEF &&reviced_tem[len-1+step]==0xFE) 
		{//检查帧头帧尾是否完整
		   
		    check_sum = 0x00;
			for(int i=2;i<len-3;i++)
			{
				check_sum += reviced_tem[i+step];
			}
			// 检验数据长度和校验码是否正确
			if(reviced_tem[len-3+step] == check_sum || reviced_tem[len-3+step]== 0xff)
				;
			else
			{
				RCLCPP_WARN(this->get_logger()," check_sum error ...");
				printf(" check_sum: 0x%02X , reviced check value: 0x%02X \n", check_sum,reviced_tem[len-3+step]);
				continue;
			}
		

			if (reviced_tem[3+step] ==0x07 ) //mickv3 4个电机 数据
			{
				i=4+step;
				for(int j=0;j<4;j++)
				{
					speed_rpm.int32_dat=0;
					total_angle.int32_dat =0;
					round_cnt.int32_dat=0;

					speed_rpm.byte_data[3]=reviced_tem[i++]; 
					speed_rpm.byte_data[2]=reviced_tem[i++];
					speed_rpm.byte_data[1] = reviced_tem[i++] ; 
					speed_rpm.byte_data[0] = reviced_tem[i++] ;
					mick_chassis.moto_measurements[j].speed_rpm = speed_rpm.int32_dat;			//*1000	
					
					round_cnt.byte_data[3]=reviced_tem[i++]; 
					round_cnt.byte_data[2]=reviced_tem[i++];
					round_cnt.byte_data[1] = reviced_tem[i++] ; 
					round_cnt.byte_data[0] = reviced_tem[i++] ;
					mick_chassis.moto_measurements[j].round_cnt =  round_cnt.int32_dat;
					
					total_angle.byte_data[3]=reviced_tem[i++]; 
					total_angle.byte_data[2]=reviced_tem[i++];
					total_angle.byte_data[1] = reviced_tem[i++] ; 
					total_angle.byte_data[0] = reviced_tem[i++] ;
					mick_chassis.moto_measurements[j].angle =  total_angle.int32_dat;

					mick_chassis.moto_measurements[j].available = 0x01;
				}
				
				// ROS_INFO_STREAM("recived mickv3 chassis motor data" ); 
				// for(j=0;j<4;j++)
				// {
				// 	// 打印四个电机的转速、转角、温度等信息
				// 	ROS_INFO_STREAM("M "<< j <<"\t rpm: "<<mick_chassis.moto_measurements[j].speed_rpm
				// 							<<": \t round_cnt: "<<mick_chassis.moto_measurements[j].round_cnt
				// 							<<"  angle: "<<mick_chassis.moto_measurements[j].angle );
				// }
			}
			else if (reviced_tem[3+step] ==0x08 ) //mickv3 4个电机状态数据
			{
				;
			}
			else if (reviced_tem[3+step] ==0xA7 ) //mickv3 里程计
			{
				i=4+step;
				odom.int16_dat = 0;
				odom.int16_dat=0;odom.byte_data[1] = reviced_tem[i++] ; odom.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.odom_measurements.vx = odom.int16_dat/1000.0f;

				odom.int16_dat = 0;
				odom.int16_dat=0;odom.byte_data[1] = reviced_tem[i++] ; odom.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.odom_measurements.vy = odom.int16_dat/1000.0f;

				odom.int16_dat = 0;
				odom.int16_dat=0;odom.byte_data[1] = reviced_tem[i++] ; odom.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.odom_measurements.wz = odom.int16_dat/1000.0f;
				 
				mick_chassis.odom_measurements.available = 0x01;
				//printf("odom: %f\t%f\t%f\n",mick_chassis.odom_measurements.vx,
				//							mick_chassis.odom_measurements.vy,
				//							mick_chassis.odom_measurements.wz);
			}
			else if (reviced_tem[3+step] ==0xA0 ) // IMU 数据
			{
				i=4+step;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ;imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.ax = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.ay = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.az = imu.int16_dat;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ;imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.gx = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.gy = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.gz = imu.int16_dat;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.mx = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.my = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.mz = imu.int16_dat;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.pitch = imu.int16_dat/100.0f;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.roll = imu.int16_dat/100.0f;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				mick_chassis.imu_measurements.yaw = imu.int16_dat/100.0f;
				mick_chassis.imu_measurements.available = 0x01;
				publish_imu(mick_chassis.imu_measurements);
				//printf("recived imu  data \n" ); 
			}
			else if (reviced_tem[3+step] ==0xA1 )
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
					mick_chassis.ultrasonic.push_back(ultra_tem);
				}
				printf("recived Ulrat data" ); 
			}
			else if (reviced_tem[3 + step] == 0xA3)
			{
				i = 4 + step;

				mick_chassis.rc.ch1 = reviced_tem[i++];
				mick_chassis.rc.ch1 = (mick_chassis.rc.ch1 << 8) + reviced_tem[i++];
				mick_chassis.rc.ch2 = reviced_tem[i++];
				mick_chassis.rc.ch2 = (mick_chassis.rc.ch2 << 8) + reviced_tem[i++];
				mick_chassis.rc.ch3 = reviced_tem[i++];
				mick_chassis.rc.ch3 = (mick_chassis.rc.ch3 << 8) + reviced_tem[i++];
				mick_chassis.rc.ch4 = reviced_tem[i++];
				mick_chassis.rc.ch4 = (mick_chassis.rc.ch4 << 8) + reviced_tem[i++];
				mick_chassis.rc.sw1 = reviced_tem[i++];
				mick_chassis.rc.sw2 = reviced_tem[i++];
				mick_chassis.rc.sw3 = reviced_tem[i++];
				mick_chassis.rc.sw4 = reviced_tem[i++];
				mick_chassis.rc.type = reviced_tem[i++];// 1 DJI-DBUS   2 SBUS 遥控器类型
				mick_chassis.rc.status = reviced_tem[i++];
				mick_chassis.rc.update = 0x01;
				if (mick_chassis.rc.ch1 >= (RC_MIN-200) && mick_chassis.rc.ch1 <=(RC_MAX+200))
				{
					mick_chassis.rc.available = 0x01;
				}
				else
				{
					printf("mick_chassis.rc.chx < RC_MIN || mick_chassis.rc.chx > RC_MAX");
				}
				// if(show_message)
				// {
				// 	ROS_INFO_STREAM("RC_Remotes date  ch1: " << mick_chassis.rc.ch1 << " ch2: " << mick_chassis.rc.ch2
				// 					 << " ch3: " << mick_chassis.rc.ch3 << " ch4: " << mick_chassis.rc.ch4 << " sw1: " 
				// 					 << rc.sw1 << " sw2: " << rc.sw2<< " sw3: " 
				// 					 << rc.sw3 << " sw4: " << rc.sw4<< " type: " << rc.type);
				// }
				return true;
			}
			else if (reviced_tem[3+step] ==0xAC ) // IO状态
			{
				;
			}
			else
			{
				printf("unrecognize frame 0x%x \n",reviced_tem[3 + step]);
			}
			//return  true;
		}
		else
		{
		  printf("frame head is wrong \n" ); 
		  
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
// 差速底盘  速度计算    安普斯电机
// 仅仅只是前轮转向模式
void calculate_chassisDiffX4_position_for_odometry(void)
{
    rclcpp::Clock clock;
 
	float linear_x,linear_y,linear_w;
		
	if(motor_init_flag == 0 && (mick_chassis.moto_measurements[0].counter ==0))
	{
		position_x = 0 ; 
		position_y =0 ; 
		position_w =0 ; 
		curr_time = clock.now().seconds();

		last_time =  curr_time;
		motor_init_flag++;//保证程序只进入一次
		return ;
	}

	if(mick_chassis.odom_measurements.available = 0x01) //直接使用底盘上传的里程计数据
	{
		linear_x = mick_chassis.odom_measurements.vx;
		linear_y = 0;
		linear_w = mick_chassis.odom_measurements.wz;  
	}

	//设置死区
	float linear_x_min = 0.01;// m/s
	float linear_w_min = 0.001;// rad/s 

	if(abs(linear_x)<linear_x_min)
	{
		linear_x=0;	
	}
	if(abs(linear_w)<linear_w_min)
	{
		linear_w=0;	
	}

	curr_time = clock.now().seconds();
	double dt = curr_time - last_time;
	if(dt>1)
		dt = 0;
	last_time =  curr_time;

	//ROS_INFO_STREAM(" calculate_chassisDiffX4_position_for_odometry dt:  "<<dt<<"  vx:  "<<linear_x<<"   vy: " <<linear_y<<"   vw: " <<linear_w);

   
	position_x=position_x+cos(position_w)*linear_x*dt;
	position_y=position_y+sin(position_w)*linear_x*dt;
	position_w=position_w+linear_w*dt;
 

	if(position_w>2*WHEEL_PI)
	{
		position_w=position_w-2*WHEEL_PI;	
	}
	else if(position_w<-2*WHEEL_PI)
	{
		position_w=position_w+2*WHEEL_PI;
	}
	else;

 	//ROS_INFO_STREAM("  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w); 
    publish_odomtery(position_x,position_y,position_w,linear_x,linear_y,linear_w);
    
}
// Ackerman底盘  速度计算
// 仅仅只是前轮转向模式
void calculate_chassisAckermann_position_for_odometry(void)
{
  rclcpp::Clock clock;
 
  float linear_x,linear_y,linear_w;
	 
  if(motor_init_flag == 0 &&  (mick_chassis.moto_measurements[0].counter ==0))
  {
		position_x = 0 ; 
		position_y =0 ; 
		position_w =0 ; 
		curr_time = clock.now().seconds();
		last_time =  curr_time;
		motor_init_flag++;//保证程序只进入一次
		return ;
  }
    // float RPM = v*70.02556; // 轮子直径是258mm               70.02556=60/(3.1415926*0.258);
	float v_l = (mick_chassis.moto_measurements[1].speed_rpm/1000.0)/70.02556; //rpm -> m/s  
	float v_r = (mick_chassis.moto_measurements[2].speed_rpm/1000.0)/70.02556;
 
 	// 利用轮子的转速来推算
	linear_x =( v_r + v_l)/2.0;
	linear_y = 0;
	linear_w =( v_r-v_l)/0.796; // 左右侧轮距 0.796             前后轮距 0.8083

	curr_time = clock.now().seconds();
	double dt = curr_time - last_time;
	if(dt>1)
		dt = 0;
	last_time =  curr_time;

    RCLCPP_INFO_STREAM(this->get_logger(), "dt: " << dt << " vx: " << linear_x << " vy: " << linear_y << " vw: " << linear_w);

   
	position_x=position_x+cos(position_w)*linear_x*dt;
	position_y=position_y+sin(position_w)*linear_x*dt;
	position_w=position_w+linear_w*dt;

  if(position_w>2*WHEEL_PI)
  {
     position_w=position_w-2*WHEEL_PI;	
  }
  else if(position_w<-2*WHEEL_PI)
  {
     position_w=position_w+2*WHEEL_PI;
  }
  else;

 	RCLCPP_INFO_STREAM(this->get_logger(),"  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w); 
    publish_odomtery(position_x,position_y,position_w,linear_x,linear_y,linear_w);
    
}
//针对前后转向的 阿卡曼模型
void calculate_chassisAckermann2_position_for_odometry(void)
{
    rclcpp::Clock clock;
 
	float linear_x,linear_y,linear_w;
		
	if(motor_init_flag == 0 &&  (mick_chassis.moto_measurements[0].counter ==0))
	{
		position_x = 0 ; 
		position_y =0 ; 
		position_w =0 ; 
		curr_time = clock.now().seconds();
		last_time =  curr_time;
		motor_init_flag++;//保证程序只进入一次
		return ;
	}
	// float RPM = v*70.02556; // 轮子直径是258mm               70.02556=60/(3.1515926*0.258);
	float v_1 = (mick_chassis.moto_measurements[0].speed_rpm/1000.0)/70.02556;  
	float v_2 = (mick_chassis.moto_measurements[1].speed_rpm/1000.0)/70.02556; //rpm -> m/s  
	float v_3 = (mick_chassis.moto_measurements[2].speed_rpm/1000.0)/70.02556;
	float v_4 = (mick_chassis.moto_measurements[3].speed_rpm/1000.0)/70.02556;

	float theta_1 = (mick_chassis.moto_rmd_measurements[0].angle)*0.0001745; //(0.01° -> rad) 
	float theta_2 = (mick_chassis.moto_rmd_measurements[1].angle)*0.0001745;
	float theta_3 = (mick_chassis.moto_rmd_measurements[2].angle)*0.0001745;
	float theta_4 = (mick_chassis.moto_rmd_measurements[3].angle)*0.0001745;

		// 左右侧轮距 0.796    前后轮距 0.8083
	float rx = 0.796/2.0;
	float ry = 0.8083/2.0;
	float r2 = rx*rx + ry*ry;

	float m1 = rx*sin(theta_1)-ry*sin(theta_1)/(4*r2);
	float m2 = -rx*sin(theta_2)-ry*sin(theta_2)/(4*r2);
	float m3 = -rx*sin(theta_3)+ry*sin(theta_3)/(4*r2);
	float m4 = rx*sin(theta_4)+ry*sin(theta_4)/(4*r2);

	linear_x = (cos(theta_1)*v_1 + cos(theta_2)*v_2 + cos(theta_3)*v_3 + cos(theta_4)*v_4)/4.0;
	linear_y = (sin(theta_1)*v_1 + sin(theta_2)*v_2 + sin(theta_3)*v_3 + sin(theta_4)*v_4)/4.0;
	linear_w =  m1*v_1 + m2*v_2 + m3*v_3 + m4*v_4;

	// 利用轮子的转速来推算
	curr_time = clock.now().seconds();
	double dt = curr_time - last_time;
	if(dt>1)
		dt = 0;
	last_time =  curr_time;

	RCLCPP_INFO_STREAM(this->get_logger(),"  dt:  "<<dt<<"  vx:  "<<linear_x<<"   vy: " <<linear_y<<"   vw: " <<linear_w);


	position_x=position_x+cos(position_w)*linear_x*dt;
	position_y=position_y+sin(position_w)*linear_x*dt;
	position_w=position_w+linear_w*dt;

	if(position_w>2*WHEEL_PI)
	{
		position_w=position_w-2*WHEEL_PI;	
	}
	else if(position_w<-2*WHEEL_PI)
	{
		position_w=position_w+2*WHEEL_PI;
	}
	else;

 	RCLCPP_INFO_STREAM(this->get_logger(),"  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w); 
    publish_odomtery(position_x,position_y,position_w,linear_x,linear_y,linear_w);
    
}
/**
 * @function 发布里程计的数据
 * 
 */
void publish_odomtery(float  position_x,float position_y,float oriention,
								float vel_linear_x,float vel_linear_y,float vel_linear_w)
{
	rclcpp::Clock clock;
	// static tf::TransformBroadcaster odom_broadcaster_;  //定义tf对象
     

    geometry_msgs::msg::TransformStamped odom_trans;  //创建一个tf发布需要使用的TransformStamped类型消息
    geometry_msgs::msg::Quaternion odom_quat;   //四元数变量
    nav_msgs::msg::Odometry odom;  //定义里程计对象

		
	//里程计的偏航角需要转换成四元数才能发布
	odom_quat = tf2::toMsg(tf2::Quaternion(0, 0, std::sin(oriention / 2), std::cos(oriention / 2)));

	//载入坐标（tf）变换时间戳
	odom_trans.header.stamp = clock.now();
	//发布坐标变换的父子坐标系
	odom_trans.header.frame_id = "odom";     
	odom_trans.child_frame_id = "base_link";       
	//tf位置数据：x,y,z,方向
	odom_trans.transform.translation.x = position_x;
	odom_trans.transform.translation.y = position_y;
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
	odom.pose.pose.position.x = position_x;     
	odom.pose.pose.position.y = position_y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;       
	//载入线速度和角速度
	odom.twist.twist.linear.x = vel_linear_x;
	odom.twist.twist.linear.y = vel_linear_y;
	odom.twist.twist.angular.z = vel_linear_w;    
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
// 发布遥控器数据
bool publish_joy_msg(rclcpp::Node::SharedPtr node,rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_)
{
	if(mick_chassis.rc.update != 0x01)
	{
		RCLCPP_WARN_STREAM(this->get_logger(),"rc.update != 0x01 ");
		return false;
	}

	// 0 未标定   1 标定中   2标定完成
	if(rc_init_flags != 2)
	{
		RCLCPP_WARN_STREAM(this->get_logger(),"rc.state != 0x02");
		int flag = calculate_rc_offset(node);
		//ROS_WARN_STREAM("flag "<<flag<<"  rc.state "<<rc.state);
		if(flag == 0)
		{
			rc_init_flags = 0 ;
			RCLCPP_WARN_STREAM(this->get_logger(),"calculate_rc_offset failed .... ");
		}
		else if(flag == 1)
		{
			rc_init_flags =1 ;
			RCLCPP_WARN_STREAM(this->get_logger(),"initial .... ");
		}
	}
	else
	{
		if (mick_chassis.rc.available == 1) 
		{
			 
			float ch1,ch2,ch3,ch4;
			 
			float rc_k = 1;

			if (mick_chassis.rc.sw2 == 1)            rc_k = 1;
			else if (mick_chassis.rc.sw2 == 3)   rc_k = 2;
			else if (mick_chassis.rc.sw2 == 2)  	rc_k = 3; // 3m/s
			else 				rc_k = 0;
			
			// 设置死区
			ch1 = (mick_chassis.rc.ch1 - mick_chassis.rc.ch1_offset) / (RC_MAX - mick_chassis.rc.ch1_offset);
			if(abs(ch1)<0.2) ch1=0;
			ch2 =(mick_chassis.rc.ch2 - mick_chassis.rc.ch2_offset) / (RC_MAX - mick_chassis.rc.ch2_offset);
			if(abs(ch2)<0.2) ch2=0;
			ch3 = (mick_chassis.rc.ch3 - mick_chassis.rc.ch3_offset) / (RC_MAX - mick_chassis.rc.ch3_offset);
			if(abs(ch3)<0.2) ch3=0;
			ch4 =(mick_chassis.rc.ch4 - mick_chassis.rc.ch4_offset) / (RC_MAX - mick_chassis.rc.ch4_offset);
			if(abs(ch4)<0.2) ch4=0;

			sensor_msgs::msg::Joy joy_msg;
			joy_msg.axes.push_back(RC_K *rc_k*ch1);
			joy_msg.axes.push_back(RC_K *rc_k*ch2);
			joy_msg.axes.push_back(RC_K *rc_k*ch3);
			joy_msg.axes.push_back(RC_K *rc_k*ch4);
			joy_msg.buttons.push_back(mick_chassis.rc.sw1);
			joy_msg.buttons.push_back(mick_chassis.rc.sw2);
			joy_pub_->publish(joy_msg);
 
			return 1;
		}
	}
	mick_chassis.rc.update = 0;
	 
	return 1;
}
// 计算遥控器的中位值
int calculate_rc_offset(rclcpp::Node::SharedPtr node)

{
	int re_flag = 0;
	if(init_times < 20)
	{
		if ((mick_chassis.rc.ch1 > 900 && mick_chassis.rc.ch1 < 1100) && (mick_chassis.rc.ch2 > 900 && mick_chassis.rc.ch2 < 1100)
			&& (mick_chassis.rc.ch3 > 900 && mick_chassis.rc.ch3 < 1100) && (mick_chassis.rc.ch4 > 900 && mick_chassis.rc.ch4 < 1100))
		{
			sum_offset[0] += mick_chassis.rc.ch1;
			sum_offset[1] += mick_chassis.rc.ch2;
			sum_offset[2] += mick_chassis.rc.ch3;
			sum_offset[3] += mick_chassis.rc.ch4;
			mick_chassis.rc.available = 0;
			rc_init_flags = 1; // 0 未标定   1 标定中   2标定完成
			init_times++;
			RCLCPP_WARN_STREAM(this->get_logger(),"calibrate...");
			re_flag = 1;
		}
	}
	else
	{
		mick_chassis.rc.ch1_offset = sum_offset[0] / init_times;
		mick_chassis.rc.ch2_offset = sum_offset[1] / init_times;
		mick_chassis.rc.ch3_offset = sum_offset[2] / init_times;
		mick_chassis.rc.ch4_offset = sum_offset[3] / init_times;
		RCLCPP_INFO_STREAM(this->get_logger(),"ch1_offset: " <<mick_chassis.rc.ch1_offset << " ch2_offset: " << mick_chassis.rc.ch2_offset
				 << "ch3_offset: " <<mick_chassis.rc.ch3_offset << " ch4_offset: " << mick_chassis.rc.ch4_offset);
		if (mick_chassis.rc.ch1_offset == 0 || mick_chassis.rc.ch2_offset == 0 || mick_chassis.rc.ch3_offset == 0 || mick_chassis.rc.ch4_offset == 0)
		{
			mick_chassis.rc.available = 0;
			rc_init_flags = 0;
			init_times = 0;
			sum_offset[0] = 0;
			sum_offset[1] = 0;
			sum_offset[2] = 0;
			sum_offset[3] = 0;
			RCLCPP_WARN_STREAM(this->get_logger(),"calibrate faild...");
			re_flag = 0;
		}
		else
		{
			mick_chassis.rc.available = 1;
			rc_init_flags = 0x02;
			RCLCPP_INFO_STREAM(this->get_logger(),"remote calibrate successful");
			re_flag = 2; //标定成功
		}
	}
	return re_flag; //标定中
}
 


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto chassis_node = std::make_shared<MickRobotNode>();
    rclcpp::spin(chassis_node);
    rclcpp::shutdown();
    return 0;
}

