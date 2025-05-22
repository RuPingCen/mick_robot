#ifndef MICK_CHASSIS_PROTOCOL_H
#define MICK_CHASSIS_PROTOCOL_H

#include <stdint.h>
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

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include "serial/serial.h"
 




#define  WHEEL_RATIO    19.0 		// 麦克纳母轮模式 减速比 3508电机减速比为1:19
#define  WHEEL_K        0.355            // 麦克纳母轮模式

#define  WHEEL_L        0.4                 //左右轮子的间距
#define  WHEEL_D        0.17 	   	//轮子直径  6.5寸的轮子
//#define  WHEEL_R    WHEEL_D/2.0 			//轮子半径
#define  WHEEL_PI       3.141693 			//pi


namespace MickRobot
{
class MickRobotCHASSIS  
{ 
	public:
        chassis_measure_t chassis_measurements;
    
        


	private:
        //serial::Serial ros_ser;
    
        std::string dev_ = "/dev/ttyUSB0";
        int baud_ = 115200;
        double delay_ = 0; 

        std::shared_ptr<boost::asio::io_service> io_service_;
        std::shared_ptr<boost::asio::serial_port> serial_port_;

        INT32Data  motor_upload_counter,total_angle,round_cnt,speed_rpm;
        Int16Data  imu,odom;

       
        // 计算里程计 用于存储全局的位置
        float position_x=0,position_y=0,position_w=0;
        int motor_init_flag = 0;
        std::chrono::high_resolution_clock::time_point last_time, curr_time ;


        unsigned int init_times = 0;
        int sum_offset[4] = {0};
        int show_message =1;
        float RC_MIN = 0, RC_MAX = 2500, RC_K = 1; //遥控器摇杆通道输出的最小值、最大值、比例系数
    

    public:
        MickRobotCHASSIS(std::string dev,int baud,int chassis_type);
        bool readData(void);
        bool analy_uart_recive_data(std::string& str_data);

        void send_speed_to_chassis(float speed_x,float speed_y,float speed_w);

        void send_rpm_to_chassis(serial::Serial& ser_port_fd, int w1, int w2, int w3, int w4);
        void send_speed_to_X4chassis(serial::Serial& ser_port_fd,float x,float y,float w);
        void send_speed_to_X4chassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd, float x,float y,float w);
        void send_speed_to_Ackerchassis(serial::Serial& ser_port_fd,float x,float w );

        void send_speed_to_4WS4WDchassis(serial::Serial& ser_port_fd,float x,float y,float w );
            void send_rpm_to_4WS4WDchassis(serial::Serial& ser_port_fd,std::vector<float> vw);
        void clear_odometry_chassis(serial::Serial& ser_port_fd);

        void calculate_chassisDiffX4_position_for_odometry(void);
};
}

#endif 
