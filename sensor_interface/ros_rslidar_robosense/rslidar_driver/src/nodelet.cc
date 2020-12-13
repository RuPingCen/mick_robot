/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver nodelet for the RSLIDAR 3D LIDARs
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "rsdriver.h"

volatile sig_atomic_t flag = 1;

namespace rslidar_driver
{
class DriverNodelet : public nodelet::Nodelet
{
public:
  DriverNodelet() : running_(false)
  {
  }

  ~DriverNodelet()
  {
    if (running_)
    {
      NODELET_INFO("shutting down driver thread");
      running_ = false;
      deviceThread_->join();
      NODELET_INFO("driver thread stopped");
    }
  }

private:
  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running_;  ///< device thread is running
  boost::shared_ptr<boost::thread> deviceThread_;

  boost::shared_ptr<rslidarDriver> dvr_;  ///< driver implementation class
};

void DriverNodelet::onInit()
{
  // start the driver
  dvr_.reset(new rslidarDriver(getNodeHandle(), getPrivateNodeHandle()));

  // spawn device poll thread
  running_ = true;
  deviceThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));
  // NODELET_INFO("DriverNodelet onInit");
}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll()
{
  while (ros::ok() && dvr_->poll())
  {
    ros::spinOnce();
  }
}
}

// Register this plugin with pluginlib.  Names must match nodelet_velodyne.xml.
//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(rslidar_driver::DriverNodelet, nodelet::Nodelet)
