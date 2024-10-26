#include "mick_chassis_protocol.hpp"

 

//默认采用差速模式  0：差速  1-麦克纳姆轮  2: Ackermann  3:4WS4WD
void send_speed_to_chassis(serial::Serial& ser_port_fd, int chassis_type, float speed_x,float speed_y,float speed_w)
{
  float v1=0,v2=0,v3=0,v4=0;


  if(chassis_type == 0) //mick-v3 差速模式
	{
		send_speed_to_X4chassis(ser_port_fd,speed_x,speed_y,speed_w);
		return ;
	}
	else if(chassis_type == 1) // 麦克纳姆轮模式
	{
		v1 = speed_x-speed_y-WHEEL_K*speed_w;       //转化为每个轮子的线速度
		v2 = speed_x+speed_y-WHEEL_K*speed_w;
		v3 =-(speed_x-speed_y+WHEEL_K*speed_w);
		v4 =-(speed_x+speed_y+WHEEL_K*speed_w);

		v1 =v1/(WHEEL_D*WHEEL_PI)*60;    //转换为轮子的速度　-》 RPM
		v2 =v2/(WHEEL_D*WHEEL_PI)*60;
		v3 =v3/(WHEEL_D*WHEEL_PI)*60;
		v4 =v4/(WHEEL_D*WHEEL_PI)*60;
		send_rpm_to_chassis(ser_port_fd,v1,v2,v3,v4);

	}
	else if(chassis_type == 2) // 阿卡曼模式
	{
		send_speed_to_Ackerchassis(ser_port_fd,speed_x, speed_w); //直接发送目标速度 和 角速度
		//ROS_INFO_STREAM("send_speed_to_Ackerchassis: "<<speed_x<<"  "<<speed_w);
		return ;
	}
	else if(chassis_type == 3) // 4WS4WD模式
	{
		send_speed_to_4WS4WDchassis(ser_port_fd,speed_x,speed_y,speed_w);
		std::cout << "send_speed_to_4WS4WDchassis: "<<speed_x<<"  "<<speed_y<<"  "<<speed_w << std::endl;
		return ;
	}
	else
	{
		std::cout << "unknown chassis type ! " << std::endl;
	}

}



/**
 * @function  发送四个点击的转速到底盘控制器
 * ＠param w1 w2 w3 w4 表示四个电机的转速 单位　RPM
 */
void send_rpm_to_chassis(serial::Serial& ser_port_fd, int w1, int w2, int w3, int w4)
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
void send_speed_to_X4chassis(serial::Serial& ser_port_fd, float x,float y,float w)
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
// 差速小车
void send_speed_to_X4chassis(std::shared_ptr<boost::asio::serial_port> ser_port_fd, float x,float y,float w)
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
void send_speed_to_Ackerchassis(serial::Serial& ser_port_fd, float x,float w)
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

void send_speed_to_4WS4WDchassis(serial::Serial& ser_port_fd, float x,float y,float w)
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
void send_rpm_to_4WS4WDchassis(serial::Serial& ser_port_fd, std::vector<float> vw)
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

void clear_odometry_chassis(serial::Serial& ser_port_fd)
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
 