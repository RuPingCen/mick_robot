#ifndef PATH_PUB_HPP_
#define PATH_PUB_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

namespace path_tracking
{

    class PathPub : public rclcpp::Node
    {
    public:
        PathPub();

    private:
        void publishPath();
        bool loadPathFromFile(const std::string &filepath);
        std::vector<std::tuple<double, double, double>> path_points_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        std::string topic_name_;
        double publish_frequency_;
        std::string path_file_path_;
    };

} // namespace path_tracking

#endif // PATH_PUB_HPP_