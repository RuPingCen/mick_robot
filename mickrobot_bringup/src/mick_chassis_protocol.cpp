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
//#include "serial/serial.h"
#include <array>
#include "sys/time.h"
 

#include "mick_chassis_msg.h"
#include "mick_chassis_protocol.h"


using namespace std;

namespace MickRobot
{
 
MickRobotCHASSIS::MickRobotCHASSIS(std::string dev,int baud,int chassis_type)
{
	
	dev_ = dev;
	baud_ = baud;
	chassis_measurements.chassis_type = chassis_type;
	 	
	io_service_ = std::make_shared<boost::asio::io_service>();
    serial_port_ = std::make_shared<boost::asio::serial_port>(*io_service_);

	try
	{
		serial_port_->open(dev_);
	}
	catch (boost::system::system_error &error)
	{
		printf("Failed to open port %s with error %s", dev_.c_str(), error.what());
		return;
	}

	if (!serial_port_->is_open())
	{
		printf( "Failed to open serial port %s", dev_.c_str());
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
		printf("Serial port is not open");
		return;
	}
	
}
bool MickRobotCHASSIS::readData(void)
{
	// 读取数据
	uint8_t tmp[1024] ;
	std::string tmp_string;
	
	size_t bytes_read = serial_port_->read_some(boost::asio::buffer(tmp, 1024));
	if (bytes_read <= 0)
	{
		printf("No data read from serial port");
		return false;
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
		if(chassis_measurements.chassis_type == 0 || chassis_measurements.chassis_type == 1)
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
		printf(" analy uart recive data error ...");
		return false;
	}
	return true;
		
}

/**
 * @function 解析串口发送过来的数据帧
 * 成功则返回true　否则返回false
 */
bool MickRobotCHASSIS::analy_uart_recive_data(std::string& str_data)
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
				printf(" check_sum error ...");
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
					chassis_measurements.moto_measurements[j].speed_rpm = speed_rpm.int32_dat;			//*1000	
					
					round_cnt.byte_data[3]=reviced_tem[i++]; 
					round_cnt.byte_data[2]=reviced_tem[i++];
					round_cnt.byte_data[1] = reviced_tem[i++] ; 
					round_cnt.byte_data[0] = reviced_tem[i++] ;
					chassis_measurements.moto_measurements[j].round_cnt =  round_cnt.int32_dat;
					
					total_angle.byte_data[3]=reviced_tem[i++]; 
					total_angle.byte_data[2]=reviced_tem[i++];
					total_angle.byte_data[1] = reviced_tem[i++] ; 
					total_angle.byte_data[0] = reviced_tem[i++] ;
					chassis_measurements.moto_measurements[j].angle =  total_angle.int32_dat;

					chassis_measurements.moto_measurements[j].available = 0x01;
				}
				
				// ROS_INFO_STREAM("recived mickv3 chassis motor data" ); 
				// for(j=0;j<4;j++)
				// {
				// 	// 打印四个电机的转速、转角、温度等信息
				// 	ROS_INFO_STREAM("M "<< j <<"\t rpm: "<<chassis_measurements.moto_measurements[j].speed_rpm
				// 							<<": \t round_cnt: "<<chassis_measurements.moto_measurements[j].round_cnt
				// 							<<"  angle: "<<chassis_measurements.moto_measurements[j].angle );
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
				chassis_measurements.odom_measurements.vx = odom.int16_dat/1000.0f;

				odom.int16_dat = 0;
				odom.int16_dat=0;odom.byte_data[1] = reviced_tem[i++] ; odom.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.odom_measurements.vy = odom.int16_dat/1000.0f;

				odom.int16_dat = 0;
				odom.int16_dat=0;odom.byte_data[1] = reviced_tem[i++] ; odom.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.odom_measurements.wz = odom.int16_dat/1000.0f;
				
				chassis_measurements.odom_measurements.available = 0x01;
				// printf("odom: %f\t%f\t%f\n",chassis_measurements.odom_measurements.vx,
				// 							chassis_measurements.odom_measurements.vy,
				// 							chassis_measurements.odom_measurements.wz);
			}
			else if (reviced_tem[3+step] ==0xA0 ) // IMU 数据
			{
				i=4+step;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ;imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.ax = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.ay = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.az = imu.int16_dat;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ;imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.gx = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.gy = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.gz = imu.int16_dat;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.mx = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.my = imu.int16_dat;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.mz = imu.int16_dat;
				
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.pitch = imu.int16_dat/100.0f;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.roll = imu.int16_dat/100.0f;
				imu.int16_dat=0;imu.byte_data[1] = reviced_tem[i++] ; imu.byte_data[0] = reviced_tem[i++] ;
				chassis_measurements.imu_measurements.yaw = imu.int16_dat/100.0f;
				chassis_measurements.imu_measurements.available = 0x01;
				//publish_imu(chassis_measurements.imu_measurements);
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
					chassis_measurements.ultrasonic.push_back(ultra_tem);
				}
				printf("recived Ulrat data" ); 
			}
			else if (reviced_tem[3 + step] == 0xA3)
			{
				i = 4 + step;

				chassis_measurements.rc.ch1 = reviced_tem[i++];
				chassis_measurements.rc.ch1 = (chassis_measurements.rc.ch1 << 8) + reviced_tem[i++];
				chassis_measurements.rc.ch2 = reviced_tem[i++];
				chassis_measurements.rc.ch2 = (chassis_measurements.rc.ch2 << 8) + reviced_tem[i++];
				chassis_measurements.rc.ch3 = reviced_tem[i++];
				chassis_measurements.rc.ch3 = (chassis_measurements.rc.ch3 << 8) + reviced_tem[i++];
				chassis_measurements.rc.ch4 = reviced_tem[i++];
				chassis_measurements.rc.ch4 = (chassis_measurements.rc.ch4 << 8) + reviced_tem[i++];
				chassis_measurements.rc.sw1 = reviced_tem[i++];
				chassis_measurements.rc.sw2 = reviced_tem[i++];
				chassis_measurements.rc.sw3 = reviced_tem[i++];
				chassis_measurements.rc.sw4 = reviced_tem[i++];
				chassis_measurements.rc.type = reviced_tem[i++];// 1 DJI-DBUS   2 SBUS 遥控器类型
				chassis_measurements.rc.status = reviced_tem[i++];
				chassis_measurements.rc.update = 0x01;
				if (chassis_measurements.rc.ch1 >= (RC_MIN-200) && chassis_measurements.rc.ch1 <=(RC_MAX+200))
				{
					chassis_measurements.rc.available = 0x01;
				}
				else
				{
					printf("chassis_measurements.rc.chx < RC_MIN || chassis_measurements.rc.chx > RC_MAX");
				}
				// if(show_message)
				// {
				// 	ROS_INFO_STREAM("RC_Remotes date  ch1: " << chassis_measurements.rc.ch1 << " ch2: " << chassis_measurements.rc.ch2
				// 					 << " ch3: " << chassis_measurements.rc.ch3 << " ch4: " << chassis_measurements.rc.ch4 << " sw1: " 
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
void MickRobotCHASSIS::send_speed_to_chassis(float speed_x,float speed_y,float speed_w)
{
	
	if(chassis_measurements.chassis_type == 0) //mick-v3 差速模式
	{
		send_speed_to_X4chassis(serial_port_, speed_x,speed_y,speed_w);
		return ;
	}
	else if(chassis_measurements.chassis_type == 1) // 麦克纳姆轮模式
	{
		float v1=0,v2=0,v3=0,v4=0;
		v1 = speed_x-speed_y-WHEEL_K*speed_w;       //转化为每个轮子的线速度
		v2 = speed_x+speed_y-WHEEL_K*speed_w;
		v3 =-(speed_x-speed_y+WHEEL_K*speed_w);
		v4 =-(speed_x+speed_y+WHEEL_K*speed_w);

		v1 =v1/(WHEEL_D*WHEEL_PI)*60;    //转换为轮子的速度　-》 RPM
		v2 =v2/(WHEEL_D*WHEEL_PI)*60;
		v3 =v3/(WHEEL_D*WHEEL_PI)*60;
		v4 =v4/(WHEEL_D*WHEEL_PI)*60;
		//send_rpm_to_chassis(v1,v2,v3,v4);
	}
	else if(chassis_measurements.chassis_type == 2) // 阿卡曼模式
	{
		//send_speed_to_Ackerchassis(speed_x, speed_w); //直接发送目标速度 和 角速度
		//ROS_INFO_STREAM("send_speed_to_Ackerchassis: "<<speed_x<<"  "<<speed_w);
		return ;
	}
	else if(chassis_measurements.chassis_type == 3) // 4WS4WD模式
	{
		//send_speed_to_4WS4WDchassis(speed_x,speed_y,speed_w);
		// RCLCPP_INFO_STREAM(node->get_logger(),"send_speed_to_4WS4WDchassis: "<<speed_x<<"  "<<speed_y<<"  "<<speed_w);
		cout << "send_speed_to_4WS4WDchassis: "<<speed_x<<"  "<<speed_y<<"  "<<speed_w << endl;
		return ;
	}
	else
	{
		// RCLCPP_INFO_STREAM(node->get_logger(),"unknown chassis type ! ");
		cout << "unknown chassis type ! " << endl;
	}
} 


/**
 * @function  发送四个点击的转速到底盘控制器
 * ＠param w1 w2 w3 w4 表示四个电机的转速 单位　RPM
 */
void MickRobotCHASSIS::send_rpm_to_chassis(serial::Serial& ser_port_fd, int w1, int w2, int w3, int w4)
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

	ser_port_fd.write(data_tem,counter);
}
// 差速小车
void MickRobotCHASSIS::send_speed_to_X4chassis(serial::Serial& ser_port_fd, float x,float y,float w)
{
	uint8_t data_tem[50];
	unsigned int speed_0ffset=10; //速度偏移值 10ｍ/s，把速度转换成正数发送
	unsigned char i,counter=0;	 
	unsigned int check=0;

	data_tem[counter++] =0xAE;
	data_tem[counter++] =0xEA;
	data_tem[counter++] =0x0B;
	data_tem[counter++] = 0xF3; //针对MickX4的小车使用F3 字段      针对MickM4的小车使用F2
	data_tem[counter++] =((x+speed_0ffset)*100)/256; // X
	data_tem[counter++] =((x+speed_0ffset)*100);
	data_tem[counter++] =((y+speed_0ffset)*100)/256; // Y
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
	ser_port_fd.write(data_tem,counter);
}
void MickRobotCHASSIS::send_speed_to_X4chassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd, float x,float y,float w)
{
	uint8_t data_tem[50];
	unsigned int speed_0ffset=10; //速度偏移值 10ｍ/s，把速度转换成正数发送
	unsigned char i,counter=0;	 
	unsigned int check=0;

	data_tem[counter++] =0xAE;
	data_tem[counter++] =0xEA;
	data_tem[counter++] =0x0B;
	data_tem[counter++] = 0xF3; //针对MickX4的小车使用F3 字段      针对MickM4的小车使用F2
	data_tem[counter++] =((x+speed_0ffset)*100)/256; // X
	data_tem[counter++] =((x+speed_0ffset)*100);
	data_tem[counter++] =((y+speed_0ffset)*100)/256; // Y
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
	//ser_port_fd->write_some(data_tem,counter);
	boost::asio::write(*ser_port_fd, boost::asio::buffer(data_tem, counter));
}
void MickRobotCHASSIS::send_speed_to_Ackerchassis(serial::Serial& ser_port_fd, float x,float w)
{
  uint8_t data_tem[50];
  unsigned int speed_0ffset=10; //速度偏移值 10ｍ/s，把速度转换成正数发送
  unsigned char i,counter=0;
  unsigned char  cmd;
  unsigned int check=0;
  cmd =0xF4;  
  data_tem[counter++] =0xAE;
  data_tem[counter++] =0xEA;
  data_tem[counter++] =0x0B;
  data_tem[counter++] =cmd;
  data_tem[counter++] =((x+speed_0ffset)*100)/256; // X
  data_tem[counter++] =((x+speed_0ffset)*100);
  data_tem[counter++] =0; // Y
  data_tem[counter++] =0;
  data_tem[counter++] =((w+speed_0ffset)*100)/256; //  w
  data_tem[counter++] =((w+speed_0ffset)*100);

  for(i=0;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =0xff;
   data_tem[2] =counter-2;
  data_tem[counter++] =0xEF;
  data_tem[counter++] =0xFE;
  ser_port_fd.write(data_tem,counter);
}
void MickRobotCHASSIS::send_speed_to_4WS4WDchassis(serial::Serial& ser_port_fd, float x,float y,float w)
{
  uint8_t data_tem[50];
  unsigned int speed_0ffset=10; //速度偏移值 10ｍ/s，把速度转换成正数发送
  unsigned char i,counter=0;
  unsigned char  cmd=0xF2; 
  unsigned int check=0;
  
  data_tem[counter++] =0xAE;
  data_tem[counter++] =0xEA;
  data_tem[counter++] =0x0B;
  data_tem[counter++] =cmd;
  data_tem[counter++] =((x+speed_0ffset)*100)/256; // X
  data_tem[counter++] =((x+speed_0ffset)*100);
  data_tem[counter++] =((y+speed_0ffset)*100)/256; // Y
  data_tem[counter++] =((y+speed_0ffset)*100);
  data_tem[counter++] =((w+speed_0ffset)*100)/256; // W
  data_tem[counter++] =((w+speed_0ffset)*100);
  for(i=0;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =0xff;
   data_tem[2] =counter-2;
  data_tem[counter++] =0xEF;
  data_tem[counter++] =0xFE;
  ser_port_fd.write(data_tem,counter);
}

/**********************************************************
 * @function  发送6个电机的转速到底盘控制器
 * ＠param vw 表示 (w1 v1 w2 v2 .... w6 v6)
 * 电机的转速 单位　RPM
 * 角度    单位  °度
**********************************************************/
void MickRobotCHASSIS::send_rpm_to_4WS4WDchassis(serial::Serial& ser_port_fd, std::vector<float> vw)
{
	if(vw.size()<12)
	{	
        
		printf("For 4ws4wd chasiss, the length of vm < 12");
		return;
	}

  uint8_t data_tem[50];
  unsigned int speed_0ffset=10000; //转速偏移10000转
   unsigned int theta_0ffset=360; 
  unsigned char i,counter=0;
  unsigned char  cmd;
  unsigned int check=0;
  cmd =0xFA;
  data_tem[counter++] =0xAE;
  data_tem[counter++] =0xEA;
  data_tem[counter++] =0x0B;
  data_tem[counter++] =cmd;
 
	i=0;
	for(int j=0;j<6;j++)
	{
		data_tem[counter++] =((vw[ i]+theta_0ffset)*100)/256;
		data_tem[counter++] =((vw[i++]+theta_0ffset)*100);
		data_tem[counter++] =(vw[ i]+speed_0ffset)/256; // 
		data_tem[counter++] =(vw[ i++]+speed_0ffset);
	}
 
  for(i=0;i<counter;i++)
  {
    check+=data_tem[i];
  }
  data_tem[counter++] =0xff;
   data_tem[2] =counter-2;
  data_tem[counter++] =0xEF;
  data_tem[counter++] =0xFE;
 
 ser_port_fd.write(data_tem,counter);
}

void MickRobotCHASSIS::clear_odometry_chassis(serial::Serial& ser_port_fd)
{
  uint8_t data_tem[50];
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
 
 ser_port_fd.write(data_tem,counter);
  
}





/**
 * @function 利用里程计数据实现位置估计
 * 
 */
// 差速底盘  速度计算    安普斯电机
// 仅仅只是前轮转向模式
void MickRobotCHASSIS::calculate_chassisDiffX4_position_for_odometry(void)
{
 
	float linear_x,linear_y,linear_w;
		
	if(motor_init_flag == 0 && (chassis_measurements.moto_measurements[0].counter ==0))
	{
		position_x = 0 ; 
		position_y =0 ; 
		position_w =0 ; 
		curr_time = std::chrono::high_resolution_clock::now();

		last_time =  curr_time;
		motor_init_flag++;//保证程序只进入一次
		return ;
	}

	if(chassis_measurements.odom_measurements.available = 0x01) //直接使用底盘上传的里程计数据
	{
		linear_x = chassis_measurements.odom_measurements.vx;
		linear_y = 0;
		linear_w = chassis_measurements.odom_measurements.wz;  
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

	curr_time = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - last_time);
	double dt = duration.count()/1000.0f;
	
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

	chassis_measurements.odom_measurements.x = position_x;
	chassis_measurements.odom_measurements.y = position_y;
	chassis_measurements.odom_measurements.yaw = position_w;

 	//ROS_INFO_STREAM("  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w); 
    //publish_odomtery(position_x,position_y,position_w,linear_x,linear_y,linear_w);
    
}
// Ackerman底盘  速度计算
// 仅仅只是前轮转向模式
// void MickRobotCHASSIS::calculate_chassisAckermann_position_for_odometry(void)
// {
//   rclcpp::Clock clock;
 
//   float linear_x,linear_y,linear_w;
	 
//   if(motor_init_flag == 0 &&  (chassis_measurements.moto_measurements[0].counter ==0))
//   {
// 		position_x = 0 ; 
// 		position_y =0 ; 
// 		position_w =0 ; 
// 		curr_time = clock.now().seconds();
// 		last_time =  curr_time;
// 		motor_init_flag++;//保证程序只进入一次
// 		return ;
//   }
//     // float RPM = v*70.02556; // 轮子直径是258mm               70.02556=60/(3.1415926*0.258);
// 	float v_l = (chassis_measurements.moto_measurements[1].speed_rpm/1000.0)/70.02556; //rpm -> m/s  
// 	float v_r = (chassis_measurements.moto_measurements[2].speed_rpm/1000.0)/70.02556;
 
//  	// 利用轮子的转速来推算
// 	linear_x =( v_r + v_l)/2.0;
// 	linear_y = 0;
// 	linear_w =( v_r-v_l)/0.796; // 左右侧轮距 0.796             前后轮距 0.8083

// 	curr_time = clock.now().seconds();
// 	double dt = curr_time - last_time;
// 	if(dt>1)
// 		dt = 0;
// 	last_time =  curr_time;

//     RCLCPP_INFO_STREAM(this->get_logger(), "dt: " << dt << " vx: " << linear_x << " vy: " << linear_y << " vw: " << linear_w);

   
// 	position_x=position_x+cos(position_w)*linear_x*dt;
// 	position_y=position_y+sin(position_w)*linear_x*dt;
// 	position_w=position_w+linear_w*dt;

//   if(position_w>2*WHEEL_PI)
//   {
//      position_w=position_w-2*WHEEL_PI;	
//   }
//   else if(position_w<-2*WHEEL_PI)
//   {
//      position_w=position_w+2*WHEEL_PI;
//   }
//   else;

//  	//RCLCPP_INFO_STREAM(this->get_logger(),"  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w); 
//    // publish_odomtery(position_x,position_y,position_w,linear_x,linear_y,linear_w);
    
// }
// //针对前后转向的 阿卡曼模型
// void MickRobotCHASSIS::calculate_chassisAckermann2_position_for_odometry(void)
// {
//     rclcpp::Clock clock;
 
// 	float linear_x,linear_y,linear_w;
		
// 	if(motor_init_flag == 0 &&  (chassis_measurements.moto_measurements[0].counter ==0))
// 	{
// 		position_x = 0 ; 
// 		position_y =0 ; 
// 		position_w =0 ; 
// 		curr_time = clock.now().seconds();
// 		last_time =  curr_time;
// 		motor_init_flag++;//保证程序只进入一次
// 		return ;
// 	}
// 	// float RPM = v*70.02556; // 轮子直径是258mm               70.02556=60/(3.1515926*0.258);
// 	float v_1 = (chassis_measurements.moto_measurements[0].speed_rpm/1000.0)/70.02556;  
// 	float v_2 = (chassis_measurements.moto_measurements[1].speed_rpm/1000.0)/70.02556; //rpm -> m/s  
// 	float v_3 = (chassis_measurements.moto_measurements[2].speed_rpm/1000.0)/70.02556;
// 	float v_4 = (chassis_measurements.moto_measurements[3].speed_rpm/1000.0)/70.02556;

// 	float theta_1 = (chassis_measurements.moto_rmd_measurements[0].angle)*0.0001745; //(0.01° -> rad) 
// 	float theta_2 = (chassis_measurements.moto_rmd_measurements[1].angle)*0.0001745;
// 	float theta_3 = (chassis_measurements.moto_rmd_measurements[2].angle)*0.0001745;
// 	float theta_4 = (chassis_measurements.moto_rmd_measurements[3].angle)*0.0001745;

// 		// 左右侧轮距 0.796    前后轮距 0.8083
// 	float rx = 0.796/2.0;
// 	float ry = 0.8083/2.0;
// 	float r2 = rx*rx + ry*ry;

// 	float m1 = rx*sin(theta_1)-ry*sin(theta_1)/(4*r2);
// 	float m2 = -rx*sin(theta_2)-ry*sin(theta_2)/(4*r2);
// 	float m3 = -rx*sin(theta_3)+ry*sin(theta_3)/(4*r2);
// 	float m4 = rx*sin(theta_4)+ry*sin(theta_4)/(4*r2);

// 	linear_x = (cos(theta_1)*v_1 + cos(theta_2)*v_2 + cos(theta_3)*v_3 + cos(theta_4)*v_4)/4.0;
// 	linear_y = (sin(theta_1)*v_1 + sin(theta_2)*v_2 + sin(theta_3)*v_3 + sin(theta_4)*v_4)/4.0;
// 	linear_w =  m1*v_1 + m2*v_2 + m3*v_3 + m4*v_4;

// 	// 利用轮子的转速来推算
// 	curr_time = clock.now().seconds();
// 	double dt = curr_time - last_time;
// 	if(dt>1)
// 		dt = 0;
// 	last_time =  curr_time;

// 	RCLCPP_INFO_STREAM(this->get_logger(),"  dt:  "<<dt<<"  vx:  "<<linear_x<<"   vy: " <<linear_y<<"   vw: " <<linear_w);


// 	position_x=position_x+cos(position_w)*linear_x*dt;
// 	position_y=position_y+sin(position_w)*linear_x*dt;
// 	position_w=position_w+linear_w*dt;

// 	if(position_w>2*WHEEL_PI)
// 	{
// 		position_w=position_w-2*WHEEL_PI;	
// 	}
// 	else if(position_w<-2*WHEEL_PI)
// 	{
// 		position_w=position_w+2*WHEEL_PI;
// 	}
// 	else;

//  	//RCLCPP_INFO_STREAM(this->get_logger(),"  position_x:  "<<position_x<<"  position_y:  "<<position_y<<"   position_w: " <<position_w); 
//    //publish_odomtery(position_x,position_y,position_w,linear_x,linear_y,linear_w);
    
// }
 
 
}