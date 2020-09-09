/**
 * 
 * 函数功能：只采集 iai 发布rgb 数据　并把串口的IMU 数据读存放
 * 
 * 
 * 分隔符为　逗号'，'　　
 * 时间戳单位为秒(s)　精确到小数点后６位(us)
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

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>//将ROS下的sensor_msgs/Image消息类型转化成cv::Mat。
#include<sensor_msgs/image_encodings.h>//头文件sensor_msgs/Image是ROS下的图像的类型，这个头文件中包含对图像进行编码的函数

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>


 #include <opencv2/imgproc/imgproc.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <boost/asio.hpp>                  //包含boost库函数
#include <boost/bind.hpp>
#include <sys/time.h>

#include<iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace cv;
using namespace boost::asio;           //定义一个命名空间，用于后面的读写操作

string topic1_name = "/kinect2/qhd/image_color";


char successed_flag1 =0,successed_flag2=0;

string filename_imudata="/home/crp/recordData/RGBD/imudata.txt";
string filename_rgbdata="/home/crp/recordData/RGBD/rgbdata.txt";
string save_imagedata = "/home/crp/recordData/RGBD/";


ofstream fout_imu;
ofstream  fout_rgb;
string rgb_str, dep_str;

bool successedFlag ;
int errorCounter=0;// number of error times

struct timeval time_val; //time varible
struct timezone tz;
double time_stamp;

bool display_IMU5211( unsigned char buf[21] ,timeval time_stamp,string &out_result);
void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue);
void  callback_function_color( const sensor_msgs::Image::ConstPtr  image_data);

int main(int argc,char** argv)
{
    string out_result;
    unsigned char buf[21];                      //定义字符串长度
    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));

    //namedWindow("image color",CV_WINDOW_AUTOSIZE);
    ros::init(argc,argv,"kinect2_listen");
    if(!ros::ok())
             return 0;
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe(topic1_name,50,callback_function_color);

    ros::AsyncSpinner spinner(2); // Use 2 threads
    spinner.start();

    fout_imu.open(filename_imudata.c_str(),ios::trunc); //文件不存在时候自动创建
    fout_rgb.open(filename_rgbdata.c_str(),ios::trunc);
    if(!fout_imu.is_open())
    {
            cerr<<filename_imudata<<" file not exist"<<endl;
    }
    if(!fout_rgb.is_open())
    {
            cerr<<filename_rgbdata<<" file not exist"<<endl;
    }


    while(ros::ok())
    {

			//   write(sp, buffer("Hello world", 12));//read and write can't use in same time?

			int num = read (sp,buffer(buf));//
			// cout<< num <<"get string "<<buf<<endl;
			gettimeofday(&time_val,&tz);//us
			// time_stamp =time_val.tv_sec+ time_val.tv_usec/1000000.0;
			//  cout<<"time:" <<  time_stamp<<endl;
			if(num == 21)
			{
                successedFlag = display_IMU5211(buf,time_val,out_result);
				fout_imu<<out_result;
			}
            if(successedFlag == false)
                 errorCounter++;

            if(errorCounter > 20)
            {
                    errorCounter =0;
                    return 0;
            }
                waitKey(10);
			
    }
    fout_imu.close();
    fout_rgb.close();
    cout<<" EXIT ..."<<endl;
    ros::waitForShutdown();
    ros::shutdown();

    return 1;

}
void  callback_function_color(const sensor_msgs::Image::ConstPtr  image_data)
{    
   cv_bridge::CvImageConstPtr pCvImage;// 声明一个CvImage指针的实例
    //   cout<<"the frame_id:"<<image_data->frame_id.c_str()<<endl;
//    cout<<"the image heigh"<<image_data->height<<endl;
//    cout<<"the image width"<<image_data->width<<endl;
//    cout<<"the image step"<<image_data->step<<endl;
//    cout<<"listen ...."<<endl;
    Mat rgb;
	pCvImage = cv_bridge::toCvShare(image_data, image_data->encoding);//将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针
	pCvImage->image.copyTo(rgb);

	gettimeofday(&time_val,&tz);//us
	//  time_stamp =time_val.tv_sec+ time_val.tv_usec/1000000.0;
	ostringstream os_rgb;
	os_rgb<<time_val.tv_sec<<"."<<time_val.tv_usec;
	rgb_str = save_imagedata+"rgb/"+os_rgb.str()+".png";
	imwrite(rgb_str,rgb);
	fout_rgb<<os_rgb.str()<<",rgb/"<<os_rgb.str()<<".png\n";
   // imshow("image color",rgb);
	cout<<"rgb -- time:  " <<  time_val.tv_sec<<"."<<time_val.tv_usec<<endl;
    //waitKey(1);

}

bool display_IMU5211( unsigned char buf[21] ,timeval time_stamp,string &out_result)
{
        float ax,ay,az,gx,gy,gz,tempture;
        int count;
        short int temp; //需要一个16位的变量来存储这个带符号的数据

        unsigned char check_valu=0x00;
        for(int index =0;index < 20;index++)
        {
            check_valu +=buf[index];
        }
        if(buf[0] != 0xaa || buf[1]!=0x55 || check_valu!=buf[20])
        {
            cout<<"IMU5211 Uart data transmission error ....."<<endl;
            return false;
        }

        count = ((buf[5]*256+buf[4])*256+buf[3])*256+buf[2];

            temp = ( buf[6]+buf[7]*256 );
        gx = temp*200/32768.0;
            temp =( buf[8]+buf[9]*256 );
        gy = temp*200/32768.0;
            temp =( buf[10]+buf[11]*256 );
        gz = temp*200/32768.0;

            temp = ( buf[12]+buf[13]*256) ;
        ax = temp*2/32768.0;
            temp = ( buf[14]+buf[15]*256) ;
        ay = temp*2/32768.0;
            temp = ( buf[16]+buf[17]*256) ;
        az = temp*2/32768.0;

        tempture = ( buf[18]+buf[19]*256 )/10.0;

        cout<<"time_stamp: "<< time_stamp.tv_sec<<"."<<time_stamp.tv_usec<<"  gx: "<<gx<<"  gy: "<<gy<<"  gz: "<<gz<<"  ax: "<<ax<<"  ay: "<<ay<<"  az: "<<az<<"  tempture: "<<tempture<<endl;
      //     sprintf(rgb_str,"~/recordData/RGBD/rgb/%d.png",time_stamp);
      // out_result <<time_stamp<<","<<gx<<","<<gy<<","<<gz<<","<<ax<<","<<ay<<","<<az<<","<<tempture;

        ostringstream os;
        os<<time_stamp.tv_sec<<"."<<time_stamp.tv_usec<<","<<gx<<","<<gy<<","<<gz<<","<<ax<<","<<ay<<","<<az<<","<<tempture<<'\n';
        out_result =os.str();
        return true;
}

