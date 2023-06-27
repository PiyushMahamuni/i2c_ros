#include <i2c_ros/I2CAxisServer.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    const char* path {"/home/piyush/ros2_ws/src/i2c_ros/conf/sample.txt"};
    auto node {std::make_shared<I2CAxisServer>("/dev/i2c-1", path)};
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}