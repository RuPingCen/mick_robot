#ifndef MICK_CHASSIS_PROTOCOL_HPP
#define MICK_CHASSIS_PROTOCOL_HPP

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


 


void send_speed_to_chassis(serial::Serial& ser_port_fd, int chassis_type, float speed_x,float speed_y,float speed_w);

void send_rpm_to_chassis(serial::Serial& ser_port_fd, int w1, int w2, int w3, int w4);


void send_speed_to_X4chassis(serial::Serial& ser_port_fd,float x,float y,float w);
void send_speed_to_X4chassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd, float x,float y,float w);


void send_speed_to_4WS4WDchassis(serial::Serial& ser_port_fd,float x,float y,float w );
void send_speed_to_Ackerchassis(serial::Serial& ser_port_fd,float x,float w );
void send_rpm_to_4WS4WDchassis(serial::Serial& ser_port_fd,std::vector<float> vw);
void clear_odometry_chassis(serial::Serial& ser_port_fd);

 
 

#endif 
