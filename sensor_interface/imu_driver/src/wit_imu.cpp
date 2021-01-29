/**
 * @function 读取并发布wit智能的IMU数据
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

imu_measure_t wit_imu;
 

void analy_IMU_Frame(uint8_t data_raw[], int data_length);
void publish_IMU_Raw_Data(int flag);

int main(int argc,char** argv)
{
	string out_result;
  
	string sub_cmdvel_topic,pub_odom_topic,dev;
	int buad,time_out,hz;
	ros::init(argc, argv, "wit_imu");
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
	pub = n.advertise<sensor_msgs::Imu>("/wit/imu", 1);
	pub_mag = n.advertise<sensor_msgs::MagneticField>("/wit/mag", 1);
	pub_gps = n.advertise<sensor_msgs::NavSatFix>("/wit/gps", 1);

	 

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
	if(data_raw[kk] == 0x55 && data_raw[kk + 1] == 0x51)
	{
		int16_t ax = (data_raw[kk + 3]<<8|data_raw[kk + 2]);
		int16_t ay = (data_raw[kk + 5]<<8|data_raw[kk + 4]);
		int16_t az = (data_raw[kk + 7]<<8|data_raw[kk + 6]);
		float tem = (data_raw[kk + 9]<<8|data_raw[kk + 8])/100.0f;

		uint8_t sum=0x00;
		for(int i=0;i<10;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + 10])
		{
			//cout<<"ax: "<<ax<<"\t ay: "<<ay<<"\t az: "<<az<<endl;
			wit_imu.ax = ax;
			wit_imu.ay = ay;
			wit_imu.az = az;
			wit_imu.temp = tem;
			flag = 0x01;
			wit_imu.IMUFlag |=0x01;
		}

	}
	else if(data_raw[kk] == 0x55 && data_raw[kk + 1] == 0x52)
	{
		int16_t gx = (data_raw[kk + 3]<<8|data_raw[kk + 2]);
		int16_t gy = (data_raw[kk + 5]<<8|data_raw[kk + 4]);
		int16_t gz = (data_raw[kk + 7]<<8|data_raw[kk + 6]);
		//float tem = (data_raw[kk + 9]<<8|data_raw[kk + 8])/100.0f;

		uint8_t sum=0x00;
		for(int i=0;i<10;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + 10])
		{
			//cout<<"gx: "<<gx<<"\t gy: "<<gy<<"\t gz: "<<gz<<endl;
			wit_imu.gx = gx;
			wit_imu.gy = gy;
			wit_imu.gz = gz;
			wit_imu.IMUFlag |=0x02;
			flag = 0x01;
		}

	}
	else if(data_raw[kk] == 0x55 && data_raw[kk + 1] == 0x54)
	{
		int16_t mx = (data_raw[kk + 3]<<8|data_raw[kk + 2]);
		int16_t my = (data_raw[kk + 5]<<8|data_raw[kk + 4]);
		int16_t mz = (data_raw[kk + 7]<<8|data_raw[kk + 6]);
		//float tem = (data_raw[kk + 9]<<8|data_raw[kk + 8])/100.0f;

		uint8_t sum=0x00;
		for(int i=0;i<10;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + 10])
		{
			//cout<<"mx: "<<mx<<"\t my: "<<my<<"\t mz: "<<mz<<endl;
			wit_imu.mx = mx;
			wit_imu.my = my;
			wit_imu.mz = mz;
			wit_imu.IMUFlag |=0x04;
			flag = 0x01;
		}

	}
	else if(data_raw[kk] == 0x55 && data_raw[kk + 1] == 0x53)
	{
		int16_t roll = (data_raw[kk + 3]<<8|data_raw[kk + 2]);
		int16_t pitch = (data_raw[kk + 5]<<8|data_raw[kk + 4]);
		int16_t yaw = (data_raw[kk + 7]<<8|data_raw[kk + 6]);
		//float tem = (data_raw[kk + 9]<<8|data_raw[kk + 8])/100.0f;

		uint8_t sum=0x00;
		for(int i=0;i<10;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + 10])
		{
			ROS_INFO_STREAM("roll: "<<roll*180/32768.0f<<"\t pitch: "<<pitch*180/32768.0f<<"\t yaw: "<<yaw*180/32768.0f);
			wit_imu.roll = roll*180/32768.0f;
			wit_imu.pitch = pitch*180/32768.0f;
			wit_imu.yaw = yaw*180/32768.0f;
			wit_imu.IMUFlag |=0x08;
			flag = 0x01;
		}

	}
	else if(data_raw[kk] == 0x55 && data_raw[kk + 1] == 0x55)
	{
		int16_t state0 = (data_raw[kk + 3]<<8|data_raw[kk + 2]);
		int16_t state1 = (data_raw[kk + 5]<<8|data_raw[kk + 4]);
		int16_t state2 = (data_raw[kk + 7]<<8|data_raw[kk + 6]);
		int16_t state3 = (data_raw[kk + 9]<<8|data_raw[kk + 8]);

		uint8_t sum=0x00;
		for(int i=0;i<10;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + 10])
		{
			wit_imu.state0 = state0;
			wit_imu.state1 = state1;
			wit_imu.state2 = state2;
			wit_imu.state3 = state3;
			wit_imu.IMUFlag |=0x20;
			flag = 0x01;
		}

	}
	else if(data_raw[kk] == 0x55 && data_raw[kk + 1] == 0x56)
	{
		uint32_t P = (data_raw[kk + 5]<<24|data_raw[kk + 4]<<16|data_raw[kk + 3]<<8|data_raw[kk + 2]);
		uint32_t H = (data_raw[kk + 9]<<24|data_raw[kk + 8]<<16|data_raw[kk + 7]<<8|data_raw[kk + 6]);

		uint8_t sum=0x00;
		for(int i=0;i<10;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + 10])
		{
			wit_imu.press = P;
			wit_imu.high = H;
			flag = 0x01;
		}

	}
	else if(data_raw[kk] == 0x55 && data_raw[kk + 1] == 0x57)
	{
		uint32_t Lon = (data_raw[kk + 5]<<24|data_raw[kk + 4]<<16|data_raw[kk + 3]<<8|data_raw[kk + 2]);
		uint32_t Lat = (data_raw[kk + 9]<<24|data_raw[kk + 8]<<16|data_raw[kk + 7]<<8|data_raw[kk + 6]);

		uint8_t sum=0x00;
		for(int i=0;i<10;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + 10])
		{
			wit_imu.GPSLon = Lon/10000000.0f;///(Lon%10000000)/100000;
			wit_imu.GPSLat = Lat/10000000.0f;//(Lat%10000000)/100000;
			flag = 0x01;
		}

	}
	else if(data_raw[kk] == 0x55 && data_raw[kk + 1] == 0x58)
	{
		int16_t GPSHeight = (data_raw[kk + 3]<<8|data_raw[kk + 2]);
		int16_t GPSYaw = (data_raw[kk + 5]<<8|data_raw[kk + 4]);
		uint32_t GPSV =(data_raw[kk + 9]<<24|data_raw[kk + 8]<<16|data_raw[kk + 7]<<8|data_raw[kk + 6]);
		 
		uint8_t sum=0x00;
		for(int i=0;i<10;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + 10])
		{
			wit_imu.GPSHeight = GPSHeight/10.0f; 
			wit_imu.GPSYaw = GPSYaw/10.0f;
			wit_imu.GPSV = GPSV/1000.0f;
			flag = 0x01;
		}

	}
	else if(data_raw[kk] == 0x55 && data_raw[kk + 1] == 0x59)
	{
		int16_t q0 = (data_raw[kk + 3]<<8|data_raw[kk + 2]);
		int16_t q1 = (data_raw[kk + 5]<<8|data_raw[kk + 4]);
		int16_t q2 = (data_raw[kk + 7]<<8|data_raw[kk + 6]);
		int16_t q3 = (data_raw[kk + 9]<<8|data_raw[kk + 8]);

		uint8_t sum=0x00;
		for(int i=0;i<10;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + 10])
		{
			wit_imu.qw = q0/32768.0f; 
			wit_imu.qx = q1/32768.0f; 
			wit_imu.qy = q2/32768.0f; 
			wit_imu.qz = q3/32768.0f; 
			flag = 0x01;
			wit_imu.IMUFlag |=0x10;
		}

	}	 
	else if(data_raw[kk] == 0x55 && data_raw[kk + 1] == 0x5A)
	{
		uint16_t SN = (data_raw[kk + 3]<<8|data_raw[kk + 2]);
		uint16_t PDOP = (data_raw[kk + 5]<<8|data_raw[kk + 4]);
		uint16_t HDOP = (data_raw[kk + 7]<<8|data_raw[kk + 6]);
		uint16_t VDOP = (data_raw[kk + 9]<<8|data_raw[kk + 8]);

		uint8_t sum=0x00;
		for(int i=0;i<10;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + 10])
		{
			wit_imu.GPSSN = SN; 
			wit_imu.GPSPDOP = PDOP/100.0f; 
			wit_imu.GPSHDOP = HDOP/100.0f; 
			wit_imu.GPSVDOP = VDOP/100.0f; 
			flag = 0x01;
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

	Eigen::Vector3d ea0(wit_imu.roll * M_PI / 180.0,
		  wit_imu.pitch * M_PI / 180.0,
		  wit_imu.yaw * M_PI / 180.0);
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

	//msg.orientation.w = wit_imu.qw;
	//msg.orientation.x = wit_imu.qx;
	//msg.orientation.y = wit_imu.qy;
	//msg.orientation.z = wit_imu.qz;

	// msg.angular_velocity.x = wit_imu.gx/32768.0f*2000;
	// msg.angular_velocity.y = wit_imu.gy/32768.0f*2000;
	// msg.angular_velocity.z = wit_imu.gz/32768.0f*2000;

	// change to rad/s       0.0010652 = 2000/32768/57.3
	msg.angular_velocity.x = wit_imu.gx*0.0010652; 
	msg.angular_velocity.y = wit_imu.gy*0.0010652;
	msg.angular_velocity.z = wit_imu.gz*0.0010652;

	msg.linear_acceleration.x = wit_imu.ax/32768.0f*4;
	msg.linear_acceleration.y = wit_imu.ay/32768.0f*4;
	msg.linear_acceleration.z = wit_imu.az/32768.0f*4;
	pub.publish(msg);

	msg_mag.magnetic_field.x = wit_imu.mx;
	msg_mag.magnetic_field.y = wit_imu.my;
	msg_mag.magnetic_field.z = wit_imu.mz;
	msg_mag.header.stamp = msg.header.stamp;
	msg_mag.header.frame_id = msg.header.frame_id;
	pub_mag.publish(msg_mag);

	msg_gps.header.stamp = ros::Time::now();
	msg_gps.header.frame_id = frame_id;
	msg_gps.latitude = wit_imu.GPSLat;
	msg_gps.longitude = wit_imu.GPSLon;
	msg_gps.altitude = wit_imu.high;
	pub_gps.publish(msg_gps);

}
  
