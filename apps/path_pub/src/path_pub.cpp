#include "path_pub.hpp"
#include <filesystem>
#include <iostream>

namespace path_tracking
{

    PathPub::PathPub() : Node("path_pub")
    {
        // Declare parameters
        this->declare_parameter<std::string>("topic_name", "/reference_path");
        this->declare_parameter<double>("publish_frequency", 1.0);
        this->declare_parameter<std::string>("path_file_path", "parameters/path.txt");

        // Get parameters
        topic_name_ = this->get_parameter("topic_name").as_string();
        publish_frequency_ = this->get_parameter("publish_frequency").as_double();
        path_file_path_ = this->get_parameter("path_file_path").as_string();

        // Log parameter values
        RCLCPP_INFO(this->get_logger(), "Topic name: %s", topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publish frequency: %.2f Hz", publish_frequency_);
        RCLCPP_INFO(this->get_logger(), "Path file path: %s", path_file_path_.c_str());

        // Create publisher
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>(topic_name_, 10);

        // Load path from file
        if (!loadPathFromFile(path_file_path_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load path from file: %s", path_file_path_.c_str());
            rclcpp::shutdown();
            return;
        }

        // Calculate timer period from frequency
        double period_seconds = 1.0 / publish_frequency_;
        auto period = std::chrono::duration<double>(period_seconds);

        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&PathPub::publishPath, this));

        RCLCPP_INFO(this->get_logger(), "Path publisher node initialized successfully with %zu points", path_points_.size());
    }

    bool PathPub::loadPathFromFile(const std::string &filepath)
    {
        // Check if file exists
        if (!std::filesystem::exists(filepath))
        {
            RCLCPP_ERROR(this->get_logger(), "Path file does not exist: %s", filepath.c_str());
            return false;
        }

        std::ifstream file(filepath);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open path file: %s", filepath.c_str());
            return false;
        }

        path_points_.clear();
        std::string line;
        int line_num = 0;

        while (std::getline(file, line))
        {
            line_num++;

            // Skip empty lines and comments
            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            std::istringstream iss(line);
            double x, y, z;

            if (!(iss >> x >> y >> z))
            {
                RCLCPP_WARN(this->get_logger(), "Could not parse line %d: %s", line_num, line.c_str());
                continue;
            }

            path_points_.emplace_back(x, y, z);
        }

        file.close();

        if (path_points_.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "No valid path points loaded from file: %s", filepath.c_str());
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu path points from file: %s", path_points_.size(), filepath.c_str());
        return true;
    }

    void PathPub::publishPath()
    {
        if (path_points_.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No path points to publish");
            return;
        }

        auto path_msg = nav_msgs::msg::Path();
        path_msg.header.frame_id = "odom"; // You can make this configurable if needed
        path_msg.header.stamp = this->now();

        for (size_t i = 0; i < path_points_.size(); ++i)
        {
            auto pose_stamped = geometry_msgs::msg::PoseStamped();
            pose_stamped.header.frame_id = "odom"; // Same as above
            pose_stamped.header.stamp = this->now();
            // pose_stamped.header.seq = static_cast<int>(i);

            // Set position
            auto [x, y, z] = path_points_[i];
            pose_stamped.pose.position.x = x;
            pose_stamped.pose.position.y = y;
            pose_stamped.pose.position.z = z;

            // Set orientation to identity (you can calculate proper orientation if needed)
            pose_stamped.pose.orientation.w = 1.0;
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;

            path_msg.poses.push_back(pose_stamped);
        }

        path_publisher_->publish(path_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published path with %zu points", path_msg.poses.size());
    }

} // namespace path_tracking

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<path_tracking::PathPub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}