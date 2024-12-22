// License: See LICENSE file in root directory.
// Copyright(c) 2022 Oradar Corporation. All Rights Reserved.

#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#endif
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <cmath>
#include "src/ord_lidar_driver.h"
#include <sys/time.h>

using namespace std;
using namespace ordlidar;

#define Degree2Rad(X) ((X)*M_PI / 180.)
#ifdef ROS_FOUND
void publish_msg(ros::Publisher *pub, full_scan_data_st *scan_frame, ros::Time start,
                 double scan_time, std::string frame_id, bool clockwise,
                 double angle_min, double angle_max, double min_range, double max_range)
{
  sensor_msgs::LaserScan scanMsg;
  int point_nums = scan_frame->vailtidy_point_num;

  scanMsg.header.stamp = start;
  scanMsg.header.frame_id = frame_id;
  scanMsg.angle_min = Degree2Rad(scan_frame->data[0].angle);
  scanMsg.angle_max = Degree2Rad(scan_frame->data[point_nums - 1].angle);
  double diff = scan_frame->data[point_nums - 1].angle - scan_frame->data[0].angle;
  scanMsg.angle_increment = Degree2Rad(diff/point_nums);
  scanMsg.scan_time = scan_time;
  scanMsg.time_increment = scan_time / point_nums;
  scanMsg.range_min = min_range;
  scanMsg.range_max = max_range;

  scanMsg.ranges.assign(point_nums, std::numeric_limits<float>::quiet_NaN());
  scanMsg.intensities.assign(point_nums, std::numeric_limits<float>::quiet_NaN());

  float range = 0.0;
  float intensity = 0.0;
  float dir_angle = 0.0;
  unsigned int last_index = 0;
  //printf("point_nums:%d, diff:%f, angle_increment:%f\n", point_nums, diff,scanMsg.angle_increment);
  for (int i = 0; i < point_nums; i++)
  {
    range = scan_frame->data[i].distance * 0.001;
    intensity = scan_frame->data[i].intensity;

    if ((range > max_range) || (range < min_range))
    {
      range = 0.0;
      intensity = 0.0;
    }

    if (!clockwise)
    {
      dir_angle = static_cast<float>(360.f - scan_frame->data[i].angle);
    }
    else
    {
      dir_angle = scan_frame->data[i].angle;
    }

    if ((dir_angle < angle_min) || (dir_angle > angle_max))
    {
      range = 0;
      intensity = 0;
    }

    float angle = Degree2Rad(dir_angle);
    unsigned int index = (unsigned int)((angle - scanMsg.angle_min) / scanMsg.angle_increment);
    if (index < point_nums)
    {
      // If the current content is Nan, it is assigned directly
      if (std::isnan(scanMsg.ranges[index]))
      {
        scanMsg.ranges[index] = range;
        unsigned int err = index - last_index;
        if (err == 2)
        {
          scanMsg.ranges[index - 1] = range;
          scanMsg.intensities[index - 1] = intensity;
        }
      }
      else
      { // Otherwise, only when the distance is less than the current
        //   value, it can be re assigned
        if (range < scanMsg.ranges[index])
        {
          scanMsg.ranges[index] = range;
        }
      }
      scanMsg.intensities[index] = intensity;
      last_index = index;
    }
  }

  pub->publish(scanMsg);
}

#elif ROS2_FOUND
void publish_msg(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr &pub, full_scan_data_st *scan_frame, rclcpp::Time start,
                 double scan_time, std::string frame_id, bool clockwise,
                 double angle_min, double angle_max, double min_range, double max_range)
{
  sensor_msgs::msg::LaserScan scanMsg;
  int point_nums = scan_frame->vailtidy_point_num;

  scanMsg.header.stamp = start;
  scanMsg.header.frame_id = frame_id;
  scanMsg.angle_min = Degree2Rad(scan_frame->data[0].angle);
  scanMsg.angle_max = Degree2Rad(scan_frame->data[point_nums - 1].angle);
  double diff = scan_frame->data[point_nums - 1].angle - scan_frame->data[0].angle;
  scanMsg.angle_increment = Degree2Rad(diff/point_nums);
  scanMsg.scan_time = scan_time;
  scanMsg.time_increment = scan_time / point_nums;
  scanMsg.range_min = min_range;
  scanMsg.range_max = max_range;

  scanMsg.ranges.assign(point_nums, std::numeric_limits<float>::quiet_NaN());
  scanMsg.intensities.assign(point_nums, std::numeric_limits<float>::quiet_NaN());

  float range = 0.0;
  float intensity = 0.0;
  float dir_angle;
  unsigned int last_index = 0;

  for (int i = 0; i < point_nums; i++)
  {
    range = scan_frame->data[i].distance * 0.001;
    intensity = scan_frame->data[i].intensity;

    if ((range > max_range) || (range < min_range))
    {
      range = 0.0;
      intensity = 0.0;
    }

    if (!clockwise)
    {
      dir_angle = static_cast<float>(360.f - scan_frame->data[i].angle);
    }
    else
    {
      dir_angle = scan_frame->data[i].angle;
    }

    if ((dir_angle < angle_min) || (dir_angle > angle_max))
    {
      range = 0;
      intensity = 0;
    }

    float angle = Degree2Rad(dir_angle);
    unsigned int index = (unsigned int)((angle - scanMsg.angle_min) / scanMsg.angle_increment);
    if (index < point_nums)
    {
      // If the current content is Nan, it is assigned directly
      if (std::isnan(scanMsg.ranges[index]))
      {
        scanMsg.ranges[index] = range;
        unsigned int err = index - last_index;
        if (err == 2)
        {
          scanMsg.ranges[index - 1] = range;
          scanMsg.intensities[index - 1] = intensity;
        }
      }
      else
      { // Otherwise, only when the distance is less than the current
        //   value, it can be re assigned
        if (range < scanMsg.ranges[index])
        {
          scanMsg.ranges[index] = range;
        }
      }
      scanMsg.intensities[index] = intensity;
      last_index = index;
    }
  }

  pub->publish(scanMsg);
}
#endif

int main(int argc, char **argv)
{
  std::string frame_id, scan_topic;
  std::string port;
  std::string device_model;

  double min_thr = 0.0, max_thr = 0.0, cur_speed = 0.0;
  int baudrate = 230400;
  int motor_speed = 10;
  double angle_min = 0.0, angle_max = 360.0;
  double min_range = 0.05, max_range = 20.0;
  bool clockwise = false;
  uint8_t type = ORADAR_TYPE_SERIAL;
  int model = ORADAR_MS200;
#ifdef ROS_FOUND
  ros::init(argc, argv, "oradar_ros");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  nh_private.param<std::string>("port_name", port, "/dev/ttyACM0");
  nh_private.param<int>("baudrate", baudrate, 230400);
  nh_private.param<double>("angle_max", angle_max, 180.00);
  nh_private.param<double>("angle_min", angle_min, -180.00);
  nh_private.param<double>("range_max", max_range, 20.0);
  nh_private.param<double>("range_min", min_range, 0.05);
  nh_private.param<bool>("clockwise", clockwise, false);
  nh_private.param<int>("motor_speed", motor_speed, 10);
  nh_private.param<std::string>("device_model", device_model, "ms200");
  nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
  nh_private.param<std::string>("scan_topic", scan_topic, "scan");
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic, 10);

  #elif ROS2_FOUND
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("oradar_ros"); // create a ROS2 Node

    // declare ros2 param
  node->declare_parameter<std::string>("port_name", port);
  node->declare_parameter<int>("baudrate", baudrate);
  node->declare_parameter<double>("angle_max", angle_max);
  node->declare_parameter<double>("angle_min", angle_min);
  node->declare_parameter<double>("range_max", max_range);
  node->declare_parameter<double>("range_min", min_range);
  node->declare_parameter<bool>("clockwise", clockwise);
  node->declare_parameter<int>("motor_speed", motor_speed);
  node->declare_parameter<std::string>("device_model", device_model);
  node->declare_parameter<std::string>("frame_id", frame_id);
  node->declare_parameter<std::string>("scan_topic", scan_topic);

  // get ros2 param
  node->get_parameter("port_name", port);
  node->get_parameter("baudrate", baudrate);
  node->get_parameter("angle_max", angle_max);
  node->get_parameter("angle_min", angle_min);
  node->get_parameter("range_max", max_range);
  node->get_parameter("range_min", min_range);
  node->get_parameter("clockwise", clockwise);
  node->get_parameter("motor_speed", motor_speed);
  node->get_parameter("device_model", device_model);
  node->get_parameter("frame_id", frame_id);
  node->get_parameter("scan_topic", scan_topic);

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);
  #endif

  OrdlidarDriver device(type, model);
  bool ret = false;

  if (port.empty())
  {
    std::cout << "can't find lidar ms200" << std::endl;
  }
  else
  {
    device.SetSerialPort(port, baudrate);

    std::cout << "get lidar type:"  << device_model.c_str() << std::endl;
    std::cout << "get serial port:"  << port.c_str() << ", baudrate:"  << baudrate << std::endl;
    #ifdef ROS_FOUND
    while (ros::ok())
    #elif ROS2_FOUND
    while (rclcpp::ok())
    #endif
    {
      if (device.isConnected() == true)
      {
        device.Disconnect();
        std::cout << "Disconnect lidar device." << std::endl;
      }

      if (device.Connect())
      {
        std::cout << "lidar device connect succuss." << std::endl;
        break;
      }
      else
      {
        std::cout << "lidar device connecting..." << std::endl;
        sleep(1);
      }
    }

    full_scan_data_st scan_data;
    #ifdef ROS_FOUND
    ros::Time start_scan_time;
    ros::Time end_scan_time;
    #elif ROS2_FOUND
    rclcpp::Time start_scan_time;
    rclcpp::Time end_scan_time;
    #endif
    double scan_duration;

    std::cout << "get lidar scan data" << std::endl;
    std::cout << "ROS topic:" << scan_topic.c_str() << std::endl;
    
		min_thr = (double)motor_speed - ((double)motor_speed  * 0.1);
		max_thr = (double)motor_speed + ((double)motor_speed  * 0.1);
    cur_speed = device.GetRotationSpeed();
    if(cur_speed < min_thr || cur_speed > max_thr)
    {
      device.SetRotationSpeed(motor_speed);
    }
    

    #ifdef ROS_FOUND
    while (ros::ok())
    #elif ROS2_FOUND
    while (rclcpp::ok())
    #endif
    {
      #ifdef ROS_FOUND
      start_scan_time = ros::Time::now();
      #elif ROS2_FOUND
      start_scan_time = node->now();
      #endif
      ret = device.GrabFullScanBlocking(scan_data, 1000);
      #ifdef ROS_FOUND
      end_scan_time = ros::Time::now();
      scan_duration = (end_scan_time - start_scan_time).toSec();
      #elif ROS2_FOUND
      end_scan_time = node->now();
      scan_duration = (end_scan_time.seconds() - start_scan_time.seconds());
      #endif
      

      
      if (ret)
      {
        #ifdef ROS_FOUND
        publish_msg(&scan_pub, &scan_data, start_scan_time, scan_duration, frame_id,
                    clockwise, angle_min, angle_max, min_range, max_range);
        #elif ROS2_FOUND
        publish_msg(publisher, &scan_data, start_scan_time, scan_duration, frame_id,
            clockwise, angle_min, angle_max, min_range, max_range);
        #endif

      }
    }

    device.Disconnect();
    
  }

  std::cout << "publish node end.." << std::endl;
  #ifdef ROS_FOUND
  ros::shutdown();
  #elif ROS2_FOUND
  rclcpp::shutdown();
  #endif
  
  return 0;
}
