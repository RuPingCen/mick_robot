

#ifndef PROJECT_CHASSIS_4WS4WD_H
#define PROJECT_CHASSIS_4WS4WD_H

/** @file
* @brief Main header for tcp communications
*/

// System Includes
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <iostream>

#define GS_ascii					0x5347
#define motion_CmdId				0x7426
#define odometry_CmdId				0x7600
#define ultrasonic_CmdId			0x7601
#define antiCollisionBar_CmdId		0x7602
#define ultrasonicBrake_CmdId		0x7603
#define antiCollisionBarBrake_CmdId	0x7604
#define motorDriver_CmdId			0x7605

//#define led_CmdId					0x7407
#define led_CmdId					0x7503


#define cmdId_lowByteIndex			0x04
#define cmdId_highByteIndex			0x05
#define crc_lowByteIndex			0x06
#define crc_highByteIndex			0x07

#define motion_state_STOP			0x00
#define motion_state_RUN			0x01
#define motion_state_BRAKE			0x02

#define Uint16_LowByte				0x00ff
#define Uint16_HighByte				0xff00

#define motion_CmdLen				0x0D
#define motion_feedback_CmdLen		0x0C

#define odom_CmdLen					0x08
#define odom_feedback_CmdLen		0x10

#define ultrasonic_CmdLen			0x08
#define ultrasonic_feedback_CmdLen	0x18

#define antiCollisionBar_CmdLen		0x08
#define antiColBar_feedback_CmdLen	0x09

#define ultrasonic_brake_feedback_CmdLen 0x09
#define antiCol_brake_feedback_CmdLen 0x09


#define ultra_antiCol_brake_CmdLen	0x09
#define ultra_antiCol_brake_feedback_CmdLen 0x09
#define driver_CmdLen				0x09
#define driver_feedback_CmdLen		0x0D
#define led_CmdLen					0x11
#define led_feedback_CmdLen			0x09

#define drive_left_side				0x00
#define drive_right_side			0x01
				
#define MAXLEN 						1024		

unsigned short int const crc16_table[256] = {
	0x0000,  0xC0C1,  0xC181,  0x0140,  0xC301, 0x03C0, 0x0280 , 0xC241 ,
	0xC601,  0x06C0,  0x0780, 0xC741,  0x0500,  0xC5C1,  0xC481,  0x0440,
	0xCC01,  0x0CC0, 0x0D80,  0xCD41,  0x0F00,  0xCFC1,  0xCE81,  0x0E40,
	0x0A00,  0xCAC1,  0xCB81,  0x0B40,  0xC901,  0x09C0,  0x0880, 0xC841,
	0xD801,  0x18C0,  0x1980,  0xD941, 0x1B00,  0xDBC1, 0xDA81,  0x1A40,
	0x1E00,  0xDEC1,  0xDF81,  0x1F40,  0xDD01,  0x1DC0,  0x1C80,  0xDC41,
	0x1400,  0xD4C1,  0xD581,  0x1540,  0xD701,  0x17C0,  0x1680,  0xD641,
	0xD201,  0x12C0,  0x1380,  0xD341,  0x1100,  0xD1C1,  0xD081,  0x1040,
	0xF001,  0x30C0,  0x3180,  0xF141,  0x3300,  0xF3C1,  0xF281,  0x3240,
	0x3600,  0xF6C1,  0xF781,  0x3740,  0xF501,  0x35C0,  0x3480,  0xF441,
	0x3C00,  0xFCC1,  0xFD81,  0x3D40,  0xFF01,  0x3FC0,  0x3E80,  0xFE41,
	0xFA01,  0x3AC0,  0x3B80,  0xFB41,  0x3900,  0xF9C1,  0xF881,  0x3840,
	0x2800,  0xE8C1,  0xE981,  0x2940,  0xEB01,  0x2BC0, 0x2A80,  0xEA41, 
	0xEE01,  0x2EC0,  0x2F80,  0xEF41,  0x2D00,  0xEDC1,  0xEC81,  0x2C40, 
	0xE401,  0x24C0,  0x2580,  0xE541,  0x2700,  0xE7C1,  0xE681,  0x2640,
	0x2200,  0xE2C1,  0xE381,  0x2340,  0xE101,  0x21C0,  0x2080,  0xE041, 
	0xA001,  0x60C0,  0x6180,  0xA141,  0x6300,  0xA3C1,  0xA281,  0x6240, 
	0x6600,  0xA6C1,  0xA781,  0x6740,  0xA501,  0x65C0,  0x6480,  0xA441,
	0x6C00,  0xACC1,  0xAD81,  0x6D40,  0xAF01,  0x6FC0,  0x6E80,  0xAE41, 
	0xAA01,0x6AC0,  0x6B80, 0xAB41,  0x6900,  0xA9C1,  0xA881,  0x6840,
	0x7800,  0xB8C1,  0xB981,  0x7940,  0xBB01,  0x7BC0,  0x7A80,  0xBA41, 
	0xBE01,  0x7EC0,  0x7F80,  0xBF41,  0x7D00,  0xBDC1,  0xBC81,  0x7C40,
	0xB401,  0x74C0,  0x7580,  0xB541,  0x7700,  0xB7C1,  0xB681,  0x7640, 
	0x7200,  0xB2C1,  0xB381,  0x7340,  0xB101,  0x71C0,  0x7080,  0xB041, 
	0x5000,  0x90C1,  0x9181,  0x5140, 0x9301, 0x53C0, 0x5280,  0x9241, 
	0x9601,  0x56C0,  0x5780,  0x9741,  0x5500,  0x95C1,  0x9481,  0x5440, 
	0x9C01,  0x5CC0,  0x5D80,  0x9D41,  0x5F00,  0x9FC1,  0x9E81,  0x5E40, 
	0x5A00,  0x9AC1,  0x9B81,  0x5B40,  0x9901,  0x59C0,  0x5880,  0x9841,
	0x8801, 0x48C0,  0x4980, 0x8941,  0x4B00,  0x8BC1,  0x8A81,  0x4A40,
	0x4E00,  0x8EC1,  0x8F81,  0x4F40, 0x8D01,  0x4DC0,  0x4C80,  0x8C41, 
	0x4400,  0x84C1, 0x8581,  0x4540,  0x8701,  0x47C0,  0x4680,  0x8641, 
	0x8201, 0x42C0,  0x4380,  0x8341,  0x4100,  0x81C1,  0x8081,  0x4040
};
typedef struct chassisMotionCmd
{
	//motion
	float v;
	float w;
	unsigned char motion_state;
	unsigned char vel_gear;
}chassis_motion_cmd;

typedef struct chassisInfoCheckFeedback
{
	//motion
	float v;
	float w;
	
	double mileage;
	float angle;

	unsigned char update_flag; //    led_update .......   motion_update

	unsigned short int ultrasonic_distance[8];
	unsigned char antiCollisionBarStatus; 
	unsigned char ultrasonicBrakeStatus;  /* 超声紧急自动制动功能开关状态, 0-关闭, 1-开启 */
	unsigned char antiCollisionBarBrakeStatus;/* 防撞条紧急自动制动功能开关状态 0-关闭, 1-开启 */
	unsigned int driverStatus;/*   查询驱动器异常状态 */
	unsigned char ledExecuteStatus;	
}chassis_info_check_feedback_;
typedef struct ledPara_
{
	unsigned char channel;
	unsigned char mode;
	unsigned char on_lightness;
	
	unsigned char on_time;
	unsigned char off_time;
	unsigned char flash_lightness;
	
	unsigned char speed;
	unsigned char lightness_min;
	unsigned char lightness_max;
	
	
}ledParam;

/**
@brief This 
*/
unsigned short int CRC16(const unsigned char *buffer, unsigned int len);
unsigned short int uchar_to_ushort(unsigned char sign,unsigned char i);
unsigned char* write_motion_cmd(chassis_motion_cmd &motion_cmd_para);
unsigned char* read_odometry_cmd(void);
unsigned char* read_ultrasonic_cmd(void);
unsigned char* read_antiColBar_cmd(void);
unsigned char* write_ultra_brake_cmd(unsigned char cmd);
unsigned char* write_antiColBar_brake_cmd(unsigned char cmd);
unsigned char* read_driver_state_cmd(unsigned char driverSide);
unsigned char* write_led_cmd(ledParam led_para_);

void decode_motion_cmd(unsigned char* buf,unsigned char len);
short decode_odometry_cmd(unsigned char* buf,unsigned char len);
short int decode_ultrasonic_cmd(unsigned char* buf,unsigned char len);
short int decode_antiColBar_cmd(unsigned char* buf,unsigned char len);
short int decode_ultrasonic_brake_cmd(unsigned char* buf,unsigned char len);
short int decode_antiColBar_brake_cmd(unsigned char* buf,unsigned char len);
void decode_driver_exception_cmd(unsigned char* buf,unsigned char len);
void decode_led_cmd(unsigned char* buf,unsigned char len);

short int decode_cmd(unsigned char* buffer,unsigned char len);

int chassis_init(void);
short int udp_init(void);
void udp_close(void);

#endif //PROJECT_DEAL_CMD_H
