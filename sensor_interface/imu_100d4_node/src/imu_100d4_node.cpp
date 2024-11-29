#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h> 
#include <geometry_msgs/msg/transform_stamped.hpp> 
#include <eigen3/Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <array>

using namespace std::chrono_literals;

static const uint8_t stop[6] = {0xA5, 0x5A, 0x04, 0x02, 0x06, 0xAA};
static const uint8_t mode[6] = {0xA5, 0x5A, 0x04, 0x01, 0x05, 0xAA};

static const uint8_t magnetic_correct_start[6] = {0xA5, 0x5A, 0x04, 0xE2, 0xE6, 0xAA};
static const uint8_t magnetic_correct_stop[6] = {0xA5, 0x5A, 0x04, 0xE4, 0xE8, 0xAA};

// 使用 std::array 替代裸数组
static std::array<uint8_t, 200> data_raw;
static std::string frame_id;
static int data_length = 81;

static rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
static rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;
static rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;

class ImuNode : public rclcpp::Node
{
public:
    ImuNode()
        : Node("imu_node"), data_length_(0)
    {
        // 参数声明和获取
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        declare_parameter<std::string>("model", "100D4");
        declare_parameter<int>("baud", 115200);
        declare_parameter<std::string>("frame_id", "imu");
        declare_parameter<double>("delay", 0.0);
        declare_parameter<double>("gravity", 9.81);
        declare_parameter<int>("use_magnetic", 0);

        port_ = get_parameter("port").as_string();
        model_ = get_parameter("model").as_string();
        baud_ = get_parameter("baud").as_int();
        frame_id_ = get_parameter("frame_id").as_string();
        delay_ = get_parameter("delay").as_double();
        gravity_ = get_parameter("gravity").as_double();
        use_magnetic_ = get_parameter("use_magnetic").as_int();

        io_service_ = std::make_shared<boost::asio::io_service>();
        serial_port_ = std::make_shared<boost::asio::serial_port>(*io_service_);

        try
        {
            serial_port_->open(port_);
        }
        catch (boost::system::system_error &error)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open port %s with error %s", port_.c_str(), error.what());
            return;
        }

        if (!serial_port_->is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port_.c_str());
            return;
        }

        // 设置串口选项
        typedef boost::asio::serial_port_base sb;
        sb::baud_rate baud_option(baud_);
        sb::flow_control flow_control(sb::flow_control::none);
        sb::parity parity(sb::parity::none);
        sb::stop_bits stop_bits(sb::stop_bits::one);

        serial_port_->set_option(baud_option);
        serial_port_->set_option(flow_control);
        serial_port_->set_option(parity);
        serial_port_->set_option(stop_bits);

        // 发布者初始化
        pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("/imu_100d4/imu_data", 1);
        pub_mag_ = create_publisher<sensor_msgs::msg::MagneticField>("/imu_100d4/mag", 1);
        //pub_gps_ = create_publisher<sensor_msgs::msg::NavSatFix>("/100d4/gps", 1);

        // TF广播器初始化
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // 根据型号发送初始化命令
        if (model_ == "100D4" ||model_ == "100D2" )
        {
            if(use_magnetic_)
            {
                boost::asio::write(*serial_port_, boost::asio::buffer(magnetic_correct_start, sizeof(magnetic_correct_start)));
                std::this_thread::sleep_for(1000ms);     
            }
            else
            {
                boost::asio::write(*serial_port_, boost::asio::buffer(magnetic_correct_start, sizeof(magnetic_correct_stop)));
                std::this_thread::sleep_for(1000ms);     
            }
 
            boost::asio::write(*serial_port_, boost::asio::buffer(stop, sizeof(stop)));
            std::this_thread::sleep_for(1000ms);
            boost::asio::write(*serial_port_, boost::asio::buffer(mode, sizeof(mode)));
            std::this_thread::sleep_for(1000ms);
            data_length_ = 40; // 确保这里设置为正确的数据长度
        }

        RCLCPP_WARN(this->get_logger(), "Streaming Data...");
        timer_ = create_wall_timer(10ms, std::bind(&ImuNode::readData, this));
    }

private:
void readData()
{
    if (!serial_port_->is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }

    // 读取数据
    std::array<uint8_t, 1024> tmp;
    size_t bytes_read = serial_port_->read_some(boost::asio::buffer(tmp.data(), tmp.size()));
    if (bytes_read <= 0)
    {
        RCLCPP_WARN(this->get_logger(), "No data read from serial port");
        return;
    }


    if (bytes_read > data_raw.size())
    {
        RCLCPP_WARN(this->get_logger(), "Read too much data: %zu", bytes_read);

        std::copy(tmp.end() - data_raw.size(), tmp.end(), data_raw.begin());
        return;
    }

    std::copy(tmp.begin(), tmp.begin() + bytes_read, data_raw.begin());

    // 输出读取的数据以进行调试
    // RCLCPP_INFO(this->get_logger(), "Read %zu bytes", bytes_read);
    // for (size_t i = 0; i < bytes_read; ++i)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Data[%zu]: 0x%02X", i, data_raw[i]);
    // }

    bool found = false;
    sensor_msgs::msg::Imu imu_msg;  // 在这里声明 imu_msg

    for (size_t kk = 0; kk < bytes_read - 1; ++kk) 
    {
        if (model_ == "100D4" && data_raw[kk] == 0xA5 && data_raw[kk + 1] == 0x5A)
        {
            unsigned char *data = data_raw.data() + kk;
            uint8_t data_length = data[2];
            
            uint32_t checksum = 0;
              for(int i = 0; i < data_length - 1  ; ++i)
              {
                  checksum += (uint32_t) data[i+2];
              }
             uint16_t check = checksum % 256;
             uint16_t check_true = data[data_length+1];
    
            // 确保数据长度不超过已读取的字节数
            if (kk + data_length + 1 >= bytes_read)
            {
                RCLCPP_WARN(this->get_logger(), "Data length is larger than available bytes. Skipping...");
                break; 
            }

            if (check != check_true)
            {
                RCLCPP_WARN(this->get_logger(), "Check error");
                continue;
            }

            // 解析数据并发布IMU消息
            publishImuData(data, data_length, imu_msg); 

            // 发布TF变换
            publishTransform(imu_msg.header.stamp, imu_msg); 

            found = true;
        }
    }
    if (!found)
    {
        RCLCPP_WARN(this->get_logger(), "No valid data found");
    }
}



void publishImuData(unsigned char *data, uint8_t data_length, sensor_msgs::msg::Imu &imu_msg) 
{
    // 填充IMU数据
    float yaw,pitch,roll;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = frame_id_;

    // 映射到东北天坐标系
    yaw = -d2f_euler(data + 3) * M_PI / 180.0;
    pitch = -d2f_euler(data + 7) * M_PI / 180.0;
    roll = -d2f_euler(data + 5) * M_PI / 180.0;

    Eigen::Vector3d ea0(yaw,pitch,roll);

    RCLCPP_INFO(this->get_logger(), "yaw:%3f \t pitch:%3f\t roll:%3f\n",yaw*57.3f,pitch*57.3f,roll*57.3f);
    
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q;
    q = R;
    imu_msg.orientation.w = (double)q.w();
    imu_msg.orientation.x = (double)q.x();
    imu_msg.orientation.y = (double)q.y();
    imu_msg.orientation.z = (double)q.z();
        
    // 填充IMU数据
    // 与Xsens IMU 对应， 将三驰IMU 投影到东北天坐标系
    imu_msg.angular_velocity.x = -d2f_gyro(data + 15);
    imu_msg.angular_velocity.y = -d2f_gyro(data + 17);
    imu_msg.angular_velocity.z = d2f_gyro(data + 19);

    imu_msg.linear_acceleration.x = -d2f_acc(data + 9) * gravity_;
    imu_msg.linear_acceleration.y = -d2f_acc(data + 11) * gravity_;
    imu_msg.linear_acceleration.z = d2f_acc(data + 13) * gravity_;
        
    pub_imu_->publish(imu_msg);
}


void publishTransform(const rclcpp::Time &stamp, const sensor_msgs::msg::Imu &imu_msg)
{
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;  // 使用IMU消息的时间戳
    transform.header.frame_id = "base_link";  // 参考框架
    transform.child_frame_id = frame_id_;  // IMU框架
    transform.transform.translation.x = 0.0;  // 根据需要设置
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = imu_msg.orientation; // 使用IMU消息的姿态

    tf_broadcaster_->sendTransform(transform);
}

    static float d2f_acc(uint8_t a[2])
    {
        int16_t acc = (a[0] << 8) | a[1];
        return static_cast<float>(acc) / 16384.0f;
    }

    static float d2f_gyro(uint8_t a[2])
    {
        int16_t gyro = (a[0] << 8) | a[1];
        return static_cast<float>(gyro) / 32.8f;
    }

    static float d2f_mag(uint8_t a[2])
    {
        int16_t mag = (a[0] << 8) | a[1];
        return static_cast<float>(mag) / 1.0f;
    }

    static float d2f_euler(uint8_t a[2])
    {
        int16_t euler = (a[0] << 8) | a[1];
        return static_cast<float>(euler) / 10.0f;
    }

    std::string port_;
    std::string model_;
    int baud_;
    double gravity_;
    int use_magnetic_;
    std::string frame_id_;
    double delay_;
    std::shared_ptr<boost::asio::io_service> io_service_;
    std::shared_ptr<boost::asio::serial_port> serial_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    int data_length_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImuNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

