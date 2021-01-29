/**
 * @function 读取并发布第七实验室的IMU设备
 * 
 * 备注： 
 * 
 * maker:crp
 * 2020-12-06
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

imu_measure_t sevenlab_imu;
 

void analy_IMU_Frame(uint8_t data_raw[], int data_length);
void publish_IMU_Raw_Data(int flag);
int16_t sevenlab_format_conversion(int16_t input);

int main(int argc,char** argv)
{
	string out_result;
  
	string sub_cmdvel_topic,pub_odom_topic,dev;
	int buad,time_out,hz;
	ros::init(argc, argv, "sevenlab_imu");
	ros::NodeHandle n("~");


	n.param<std::string>("dev", dev, "/dev/ttyUSB0");
	n.param<int>("buad", buad, 115200);
	n.param<int>("time_out", time_out, 1000);
	n.param<int>("hz", hz, 200);

  
	ROS_INFO_STREAM("dev:   "<<dev);
	ROS_INFO_STREAM("buad:   "<<buad);
	ROS_INFO_STREAM("time_out:   "<<time_out);
	ROS_INFO_STREAM("hz:   "<<hz);
	  
	ros::Rate loop_rate(hz);
	pub = n.advertise<sensor_msgs::Imu>("/sevenlab/imu", 1);
	pub_mag = n.advertise<sensor_msgs::MagneticField>("/sevenlab/mag", 1);
	pub_gps = n.advertise<sensor_msgs::NavSatFix>("/sevenlab/gps", 1);

	 

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
	if(data_raw[kk] == 0xA5 && data_raw[kk + 1] == 0x5A)
	{
		if(data_raw[kk+2] == 0x12 && data_raw[kk + 3] == 0xA1)
		{
			int16_t roll = (data_raw[kk + 8]<<8|data_raw[kk + 9]);
			roll = sevenlab_format_conversion(roll);

			int16_t pitch = (data_raw[kk + 6]<<8|data_raw[kk + 7]);
			pitch = sevenlab_format_conversion(pitch);

			int16_t yaw = (data_raw[kk + 4]<<8|data_raw[kk + 5]);
			yaw = sevenlab_format_conversion(yaw);
		 
			int16_t H = (data_raw[kk + 10]<<8|data_raw[kk + 11]);
			H = sevenlab_format_conversion(H);
			int16_t Temp = (data_raw[kk + 12]<<8|data_raw[kk + 13]);
			Temp = sevenlab_format_conversion(Temp);
			int16_t P = (data_raw[kk + 14]<<8|data_raw[kk + 15]);
			P = sevenlab_format_conversion(P);
			uint16_t rate = (data_raw[kk + 16]<<8|data_raw[kk + 17]);
			rate = sevenlab_format_conversion(rate);

			uint8_t sum=0x00;
			for(int i=2;i<18;i++)
			{
				sum+=data_raw[kk+i];
			}

			if(sum == data_raw[kk + 18])
			{
				ROS_INFO_STREAM("roll: "<<roll/10.0f<<"\t pitch: "<<pitch/10.0f<<"\t yaw: "<<yaw/10.0f);
				sevenlab_imu.roll = roll/10.0f;
				sevenlab_imu.pitch = pitch/10.0f;
				sevenlab_imu.yaw = yaw/10.0f;

				sevenlab_imu.temp = Temp;
				sevenlab_imu.high = H;
				sevenlab_imu.press = P;
				
				sevenlab_imu.IMUFlag |=0x02;
				flag = 0x01;
			}
		}
		else if(data_raw[kk+2] == 0x16 && data_raw[kk + 3] == 0xA2)
		{
			int16_t ax = (data_raw[kk + 4]<<8|data_raw[kk + 5]);
			ax = sevenlab_format_conversion(ax);

			int16_t ay = (data_raw[kk + 6]<<8|data_raw[kk + 7]);
			ay = sevenlab_format_conversion(ay);

			int16_t az = (data_raw[kk + 8]<<8|data_raw[kk + 9]);
			az = sevenlab_format_conversion(az);

			int16_t gx = (data_raw[kk + 10]<<8|data_raw[kk + 11]);
			gx = sevenlab_format_conversion(gx);

			int16_t gy = (data_raw[kk + 12]<<8|data_raw[kk + 13]);
			gy = sevenlab_format_conversion(gy);

			int16_t gz = (data_raw[kk + 14]<<8|data_raw[kk + 15]); 
			gz = sevenlab_format_conversion(gz);

			int16_t mx = (data_raw[kk + 16]<<8|data_raw[kk + 17]);
			mx = sevenlab_format_conversion(mx);

			int16_t my = (data_raw[kk + 18]<<8|data_raw[kk + 19]);
			my = sevenlab_format_conversion(my);

			int16_t mz = (data_raw[kk + 20]<<8|data_raw[kk + 21]);
			mz = sevenlab_format_conversion(mz);

			uint8_t sum=0x00;
			for(int i=2;i<22;i++)
			{
				sum+=data_raw[kk+i];
			}

			if(sum == data_raw[kk + 22])
			{
				//cout<<"ax: "<<ax<<"\t ay: "<<ay<<"\t az: "<<az<<endl;
				sevenlab_imu.ax = ax;
				sevenlab_imu.ay = ay;
				sevenlab_imu.az = az;

				sevenlab_imu.gx = gx;
				sevenlab_imu.gy = gy;
				sevenlab_imu.gz = gz;

				sevenlab_imu.mx = mx;
				sevenlab_imu.my = my;
				sevenlab_imu.mz = mz;
 
				flag = 0x02;
				sevenlab_imu.IMUFlag |=0x01;
			}

		}
		else ;
	}
	else
	{
		flag = 0x00;
	}
	
	if(flag == 0x01)
	{
		kk = kk+21;
	}
	else if(flag == 0x01)
	{
		kk = kk+23;
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

	Eigen::Vector3d ea0(sevenlab_imu.roll * M_PI / 180.0,
		  sevenlab_imu.pitch * M_PI / 180.0,
		  sevenlab_imu.yaw * M_PI / 180.0);
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

	//msg.orientation.w = sevenlab_imu.qw;
	//msg.orientation.x = sevenlab_imu.qx;
	//msg.orientation.y = sevenlab_imu.qy;
	//msg.orientation.z = sevenlab_imu.qz;

	// msg.angular_velocity.x = sevenlab_imu.gx/32768.0f*2000;
	// msg.angular_velocity.y = sevenlab_imu.gy/32768.0f*2000;
	// msg.angular_velocity.z = sevenlab_imu.gz/32768.0f*2000;

	// change to rad/s       0.0010652 = 2000/32768/57.3
	msg.angular_velocity.x = sevenlab_imu.gx*0.0010652; 
	msg.angular_velocity.y = sevenlab_imu.gy*0.0010652;
	msg.angular_velocity.z = sevenlab_imu.gz*0.0010652;

	msg.linear_acceleration.x = sevenlab_imu.ax/32768.0f*4;
	msg.linear_acceleration.y = sevenlab_imu.ay/32768.0f*4;
	msg.linear_acceleration.z = sevenlab_imu.az/32768.0f*4;
	pub.publish(msg);

	msg_mag.magnetic_field.x = sevenlab_imu.mx;
	msg_mag.magnetic_field.y = sevenlab_imu.my;
	msg_mag.magnetic_field.z = sevenlab_imu.mz;
	msg_mag.header.stamp = msg.header.stamp;
	msg_mag.header.frame_id = msg.header.frame_id;
	pub_mag.publish(msg_mag);

	msg_gps.header.stamp = ros::Time::now();
	msg_gps.header.frame_id = frame_id;
	msg_gps.latitude = sevenlab_imu.GPSLat;
	msg_gps.longitude = sevenlab_imu.GPSLon;
	msg_gps.altitude = sevenlab_imu.high;
	pub_gps.publish(msg_gps);

}
int16_t sevenlab_format_conversion(int16_t input)
{
	int16_t tem = input;
	if(tem & 0x8000)  
		tem = 0-(tem&0x7fff);
	else
		tem = (tem&0x7fff);
	return tem;
} 
