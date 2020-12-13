/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 Robosense, Tony Zhang
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Robosense 3D LIDAR packets to PointCloud2.

*/
#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <rslidar_pointcloud/CloudNodeConfig.h>
#include "rawdata.h"

namespace rslidar_pointcloud
{
class Convert
{
public:
  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~Convert()
  {
  }

private:
  void callback(rslidar_pointcloud::CloudNodeConfig& config, uint32_t level);

  void processScan(const rslidar_msgs::rslidarScan::ConstPtr& scanMsg);

  /// Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<rslidar_pointcloud::CloudNodeConfig> > srv_;

  boost::shared_ptr<rslidar_rawdata::RawData> data_;
  ros::Subscriber rslidar_scan_;
  ros::Publisher output_;
};

}  // namespace rslidar_pointcloud
#endif
