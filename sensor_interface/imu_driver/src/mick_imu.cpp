/**
 * @function 读取并发布自制的IMU模块数据（6050+5883L）
 * 
 * 备注： 
 * 
 * maker:crp
 * 2020-11-13
 ****************************************************/

#include <deque>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Geometry> 
#include <chrono>
#include <locale>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/assert.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h> //  close
#include <string.h> //  strerror
}

#include <serial/serial.h>
#include <std_msgs/String.h>


#include <cmath>
#include <Eigen/Dense>

 
 

using namespace std;
using namespace Eigen;
 

static int data_length = 100;
int show_message=0;
boost::asio::serial_port* serial_port = 0;
 
static uint8_t data_raw[200];
static std::vector<uint8_t> buffer_;
static std::deque<uint8_t> queue_;
static std::string name, frame_id="/imu";
static sensor_msgs::Imu msg;
static sensor_msgs::MagneticField msg_mag;
static sensor_msgs::NavSatFix msg_gps;
//static int fd_ = -1;
static ros::Publisher pub, pub_mag, pub_gps;
//static uint8_t tmp[100];
 
 
serial::Serial ros_ser;

typedef struct
{
	int16_t 	ax,ay,az;
	int16_t 	gx,gy,gz;
	int16_t 	mx,my,mz;

	float roll,pitch,yaw;
	float temp;
	float qw,qx,qy,qz;
	
	uint8_t IMUFlag;
	uint8_t GPSFlag;	

	uint32_t press,high;
	double GPSLon,GPSLat;
	double GPSHeight,GPSYaw,GPSV;
	double GPSSN,GPSPDOP,GPSHDOP,GPSVDOP;

	int16_t state0,state1,state2,state3; //

}imu_measure_t;

imu_measure_t mick_imu;
 

void analy_IMU_Frame(uint8_t data_raw[], int data_length);
void publish_IMU_Raw_Data(int flag);

int main(int argc,char** argv)
{
	string out_result;
  
	string sub_cmdvel_topic,pub_odom_topic,dev;
	int buad,time_out,hz;
	ros::init(argc, argv, "mick_imu");
	ros::NodeHandle n("~");


	n.param<std::string>("dev", dev, "/dev/ttyUSB0");
	n.param<int>("buad", buad, 115200);
	n.param<int>("time_out", time_out, 1000);
	n.param<int>("hz", hz, 200);
	n.param<int>("show_message", show_message, 1);

 
	ROS_INFO_STREAM("dev:   "<<dev);
	ROS_INFO_STREAM("buad:   "<<buad);
	ROS_INFO_STREAM("time_out:   "<<time_out);
	ROS_INFO_STREAM("hz:   "<<hz);
	ROS_INFO_STREAM("show_message:   "<<show_message);
	  
	ros::Rate loop_rate(hz);
	pub = n.advertise<sensor_msgs::Imu>("/mick/imu", 1);
	pub_mag = n.advertise<sensor_msgs::MagneticField>("/mick/mag", 1);
	//pub_gps = n.advertise<sensor_msgs::NavSatFix>("/wit/gps", 1);

	 

	// 开启串口模块
	 try
	 {
	    ros_ser.setPort(dev);
	    ros_ser.setBaudrate(buad);
	    //ros_serial::Timeout to = serial::Timeout::simpleTimeout(1000);
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
 
    while(ros::ok())
    { 
		if(ros_ser.available())
		{
			//ROS_INFO_STREAM("Reading from serial port");
			std_msgs::String serial_data;
			
			serial_data.data = ros_ser.read(ros_ser.available());//获取串口数据
			data_length=serial_data.data.size();
			if(data_length<1 || data_length>500)
			{
				ros_ser.flushInput(); //清空缓冲区数据	
				ROS_INFO_STREAM("serial data is too short ,  len: " << serial_data.data.size() );
			}
			else
			{
				for(int i=0;i<data_length;i++)
				{	
					data_raw[i] =serial_data.data.at(i);
				}
				analy_IMU_Frame(data_raw, data_length);
				publish_IMU_Raw_Data(1);
				
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
 

void analy_IMU_Frame(uint8_t data_raw[], int data_length)
{
uint8_t flag=0;
for(int kk = 0; kk < data_length - 1; )
{
 	flag=0;
	if(data_raw[kk] == 0xAE && data_raw[kk + 1] == 0xEA)
	{

		uint8_t len = data_raw[kk+2];
		if(data_raw[kk+len-2] == 0xEF && data_raw[kk+len-1] == 0xFE)
		{
			uint8_t sum=0x00;
			for(int i=0;i<len;i++)
			{
				if(i == (len-3))
					continue;

				if((kk+i)>=data_length)
					return ;
	
				sum+=data_raw[kk+i];
			}

			if(sum == data_raw[kk + len-3])
			{
				// ultrasonic measutements 
				if(data_raw[kk+3] == 0xA2 )
				{
					int16_t ax = (data_raw[kk + 4]<<8|data_raw[kk + 5]);
					int16_t ay = (data_raw[kk + 6]<<8|data_raw[kk + 7]);
					int16_t az = (data_raw[kk + 8]<<8|data_raw[kk + 9]);

					int16_t gx = (data_raw[kk + 10]<<8|data_raw[kk + 11]);
					int16_t gy = (data_raw[kk + 12]<<8|data_raw[kk + 13]);
					int16_t gz = (data_raw[kk + 14]<<8|data_raw[kk + 15]);

					int16_t mx = (data_raw[kk + 16]<<8|data_raw[kk + 17]);
					int16_t my = (data_raw[kk + 18]<<8|data_raw[kk + 19]);
					int16_t mz = (data_raw[kk + 20]<<8|data_raw[kk + 21]);

					int16_t roll = (data_raw[kk + 22]<<8|data_raw[kk + 23]);
					int16_t pitch = (data_raw[kk + 24]<<8|data_raw[kk + 25]);
					int16_t yaw = (data_raw[kk + 26]<<8|data_raw[kk + 27]);

					mick_imu.ax = ax;
					mick_imu.ay = ay;
					mick_imu.az = az; 

					mick_imu.gx = gx;
					mick_imu.gy = gy;
					mick_imu.gz = gz;

					mick_imu.mx = mx;
					mick_imu.my = my;
					mick_imu.mz = mz;
					 
					mick_imu.roll = roll/100.0f;
					mick_imu.pitch = pitch/100.0f;
					mick_imu.yaw = yaw/100.0f;
	
					mick_imu.IMUFlag =0x01;
					flag = 0x01;
 
					if(show_message)
					{
						 ROS_INFO_STREAM("roll: "<<mick_imu.roll<<"\t pitch: "<<mick_imu.pitch<<"\t yaw: "<<mick_imu.yaw);
						
					}	
				}
				flag = 0x01;
			}
			else
			{
				cerr<<"check sum error "<<endl;
			}
		}
			
		 
	}
	else
	{
		flag = 0x00;
	}
	
	if(flag == 0x01)
	{
		kk = kk+11;
	}
	else
	{
		kk = kk+1;
	}
}
}
void publish_IMU_Raw_Data(int flag)
{
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = frame_id;

	Eigen::Vector3d ea0(mick_imu.roll * M_PI / 180.0,
		  mick_imu.pitch * M_PI / 180.0,
		  mick_imu.yaw * M_PI / 180.0);
	Eigen::Matrix3d R;
	R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
		* Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
		* Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

	Eigen::Quaterniond q;
	q = R;
	q.normalize();

	msg.orientation.w = (double)q.w();
	msg.orientation.x = (double)q.x();
	msg.orientation.y = (double)q.y();
	msg.orientation.z = (double)q.z();

	//msg.orientation.w = mick_imu.qw;
	//msg.orientation.x = mick_imu.qx;
	//msg.orientation.y = mick_imu.qy;
	//msg.orientation.z = mick_imu.qz;

	// msg.angular_velocity.x = mick_imu.gx/32768.0f*2000;
	// msg.angular_velocity.y = mick_imu.gy/32768.0f*2000;
	// msg.angular_velocity.z = mick_imu.gz/32768.0f*2000;

	// change to rad/s       0.0010652 = 2000/32768/57.3
	msg.angular_velocity.x = mick_imu.gx*0.0010652; 
	msg.angular_velocity.y = mick_imu.gy*0.0010652;
	msg.angular_velocity.z = mick_imu.gz*0.0010652;

	msg.linear_acceleration.x = mick_imu.ax/32768.0f*4;
	msg.linear_acceleration.y = mick_imu.ay/32768.0f*4;
	msg.linear_acceleration.z = mick_imu.az/32768.0f*4;
	pub.publish(msg);

	msg_mag.magnetic_field.x = mick_imu.mx;
	msg_mag.magnetic_field.y = mick_imu.my;
	msg_mag.magnetic_field.z = mick_imu.mz;
	msg_mag.header.stamp = msg.header.stamp;
	msg_mag.header.frame_id = msg.header.frame_id;
	pub_mag.publish(msg_mag);

	//msg_gps.header.stamp = ros::Time::now();
	//msg_gps.header.frame_id = frame_id;
	//msg_gps.latitude = mick_imu.GPSLat;
	//msg_gps.longitude = mick_imu.GPSLon;
	//msg_gps.altitude = mick_imu.high;
	//pub_gps.publish(msg_gps);

}
  
