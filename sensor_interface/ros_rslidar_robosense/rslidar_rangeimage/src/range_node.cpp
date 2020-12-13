/* 
*
*   Copyright (C) 2018 Southeast University, Suo_ivy
*   License: Modified BSD Software License Agreement
*
*/

/*  
   This ROS node converts PointCloud2 to PCL:rangeimage

*/
// #include <ros/ros.h>
#include "ToRange.h"

/** Main node entry point  **/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "range_node");
    ros::NodeHandle node;
    
    // creat range-conversion class, which subscribes to pointcloud2 and convert to rangeimage
    rslidar_rangeimage::RangeConvert convert(node, argc, argv);

    //handle callback untill shut down
    ros::spin();
    
    return 0;
}