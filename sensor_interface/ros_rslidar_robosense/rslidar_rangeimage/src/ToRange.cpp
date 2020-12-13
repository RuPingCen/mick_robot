/* 
*
*   Copyright (C) 2018 Southeast University, Suo_ivy
*   License: Modified BSD Software License Agreement
*
*/

/*  
   This ROS node converts PointCloud2 to PCL:rangeimage

*/

#include "ToRange.h"

namespace rslidar_rangeimage
{
    RangeConvert::RangeConvert(ros::NodeHandle node, int argc, char** argv) :
        rangeimagePtr(new pcl::RangeImage),
        pointcloudPtr(new pcl::PointCloud<PointType>),
        visualization(true),
        recorddata(false)
    {
        // subscribe to rslidar_pointcloud node
        // Type: sensor_msgs::PointCloud2 
        rslidar_pointcloud_ = 
                    node.subscribe("rslidar_points",10,
                    &RangeConvert::Conversion,(RangeConvert *) this, 
                    ros::TransportHints().tcpNoDelay(true));
        
        // advertise rangeimage convert from pcl::rangeimage.creatFromPointCloud()
        // Type: Image
        range_image_ = 
                    node.advertise<sensor_msgs::Image>("range_image",10);

        // rangeimage parameter initailize
        angular_resolution_x = 0.1f;
        angular_resolution_y = 0.1f;
        max_angle_width = 360.0f;
        max_angle_height = 180.0f;
        angular_rotation_x = -90.0f;
        angular_rotation_y = 0.0f;
        angular_rotation_z = 0.0f;
        position_x = 0.0f;
        position_y = 0.0f;
        position_z = 0.0f;
        
        // Parse the commandline arguments
        if (pcl::console::parse (argc, argv, "-rx", angular_resolution_x) >= 0)
            std::cout << "Setting angular resolution in x-direction to "<<angular_resolution_x<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-ry", angular_resolution_y) >= 0)
            std::cout << "Setting angular resolution in y-direction to "<<angular_resolution_y<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-aw", max_angle_width) >= 0)
            std::cout << "Setting maximum angle width range to "<<max_angle_width<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-ah", max_angle_height) >= 0)
            std::cout << "Setting maximum angle height range to "<<max_angle_height<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-arx", angular_rotation_x) >= 0)
            std::cout << "Setting angular rotation of x-axis to "<<angular_rotation_x<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-ary", angular_rotation_y) >= 0)
            std::cout << "Setting angular rotation of y-axis to "<<angular_rotation_y<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-arz", angular_rotation_z) >= 0)
            std::cout << "Setting angular rotation of z-axis to "<<angular_rotation_z<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-px", position_x) >= 0)
            std::cout << "Setting positon of x-axis to "<<position_x<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-py", position_y) >= 0)
            std::cout << "Setting position of y-axis to "<<position_y<<"deg.\n";
        if (pcl::console::parse (argc, argv, "-pz", position_z) >= 0)
            std::cout << "Setting position of z-axis to "<<position_z<<"deg.\n";
        if (pcl::console::find_argument (argc, argv, "-v") >= 0)
        {
            visualization = false;
            std::cout << "Stop the Rangeimage Visualizer\n";
        }
        if (pcl::console::find_argument (argc, argv, "-r") >= 0)
        {
            recorddata = true;
            std::cout << "Record data(png and txt) to current path\n";
        }

        if (pcl::console::find_argument (argc, argv, "-h") >= 0)
        {
            printUsage (argv[0]);
        }

        // Set rangeimage convertion parameters
        sensor_pose=Eigen::Affine3f::Identity();
        angular_resolution_x = pcl::deg2rad(angular_resolution_x);
        angular_resolution_y = pcl::deg2rad(angular_resolution_y);
        max_angle_width = pcl::deg2rad(max_angle_width);
        max_angle_height = pcl::deg2rad(max_angle_height);
        angular_rotation_x = pcl::deg2rad(angular_rotation_x);
        angular_rotation_y = pcl::deg2rad(angular_rotation_y);
        angular_rotation_z = pcl::deg2rad(angular_rotation_z);

        coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
        nosie_level = 0.0f;
        min_range = 0.0f;
        border_size = 0;
                
    }

    void RangeConvert::printUsage(const char* NodeName)
    {
        std::cout << "\n\nUsage:"<<NodeName<<"[options] <Rangeimage_config>\n\n"
                  << "Options\n"
                  << "----------------------------------\n"
                  << "-rx <float>   angular_resolution_x in degrees (default "<<angular_resolution_x<<")\n"
                  << "-ry <float>   angular_resolution_y in degrees (default "<<angular_resolution_y<<")\n"
                  << "-aw <float>   max_angle_width  in degrees (default "<<max_angle_width<<")\n"
                  << "-ah <float>   max_angle_height in degrees (default "<<max_angle_height<<")\n"
                  << "-arx <float>  angular_rotation_x in degrees (default "<<angular_rotation_x<<")\n" 
                  << "-ary <float>  angular_rotation_y in degrees (default "<<angular_rotation_y<<")\n"
                  << "-arz <float>  angular_rotation_z in degrees (default "<<angular_rotation_z<<")\n"
                  << "-px <float>   positon_x (default "<<position_x<<")\n" 
                  << "-py <float>   positon_y (default "<<position_y<<")\n"
                  << "-pz <float>   positon_z (default "<<position_z<<")\n"
                  << "-v            stop rangeimage visualizer\n"
                  << "-r            record the dataset contains png and txt\n"
                  << "-h            this help\n"
                  << "\n";
                  
    }

    void RangeConvert::Conversion(const sensor_msgs::PointCloud2::ConstPtr &pointMsg)
    {
        // Instantination
        pcl::PointCloud<PointType>& pointcloud = *pointcloudPtr;
        pcl::RangeImage& rangeimage = *rangeimagePtr;
        // pcl::io::Image& rangepclimage = *rangepclimagePtr;

        // Convert ros sensor_msgs::PointCloud2 to pcl::PointCloud
        pcl::fromROSMsg(*pointMsg, pointcloud);

        sensor_pose = Eigen::Affine3f (Eigen::Translation3f (pointcloud.sensor_origin_[0],
                                                            pointcloud.sensor_origin_[1],
                                                            pointcloud.sensor_origin_[2]))
                                                            *Eigen::Affine3f(pointcloud.sensor_orientation_);
        // set sensor_pose rotation and translation
        sensor_pose.rotate(Eigen::AngleAxisf(angular_rotation_x, Eigen::Vector3f::UnitX()));
        sensor_pose.rotate(Eigen::AngleAxisf(angular_rotation_y, Eigen::Vector3f::UnitY()));
        sensor_pose.rotate(Eigen::AngleAxisf(angular_rotation_z, Eigen::Vector3f::UnitZ()));
        sensor_pose.translation() << position_x, position_y, position_z;

        // std::cout <<sensor_pose.matrix()<<"\n"<<endl;
        
        // Creat Rangeimage from pointcloud
        rangeimage.createFromPointCloud(pointcloud, angular_resolution_x,angular_resolution_y,
                                        max_angle_width,max_angle_height,sensor_pose,
                                        coordinate_frame,nosie_level, min_range,border_size);
        // rangeimage visualization
        if(visualization)
        {
            rangevisual.showRangeImage(rangeimage);
        }

        // Record the dataset contains png and txt 
        if(recorddata)
        {
            std::stringstream filename;
            filename << "range" <<  pointMsg->header.stamp.sec;
            std::string pngname = filename.str()+".png";
            std::string rangename = filename.str()+".txt";

            ranges = rangeimage.getRangesArray();
            unsigned char* rgb_image = pcl::visualization::FloatImageUtils::getVisualImage(ranges, rangeimage.width, rangeimage.height);
            pcl::io::saveRgbPNGFile(pngname.c_str(), rgb_image, rangeimage.width, rangeimage.height);

            std::ofstream rangefile;
            rangefile.open(rangename.c_str(),ios::trunc|ios::out);
            for (int i=0; i<rangeimage.height; i++)
            {
                for(int j=0; j<rangeimage.width; j++)
                {
                    rangefile << ranges[i*rangeimage.width + j] << ", ";
                }
                rangefile <<"\n";
            }
            std::cout<<rangeimage.width<<"  "<<rangeimage.height<<endl;
            
            rangefile.close();

        }
        

    }


}