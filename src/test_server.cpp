#include <i2c_ros/I2CAxisServer.h>
#include <rclcpp/rclcpp.hpp>

const char* path {"/home/piyush/ros2_ws/src/i2c_ros/conf/sample.txt"};

class Node: public I2CAxisServer
{
public:
    Node() : I2CAxisServer{"/dev/i2c-1", path}
    {
        t = std::thread(std::bind(&Node::keepUpdatingTf, this));
    }
private:
    std::thread t;

    void keepUpdatingTf()
    {
        rclcpp::Rate r{3};
        while(rclcpp::ok())
        {
            RCLCPP_INFO(get_logger(), "calling sendAllTransforms()");
            sendAllTransforms();
            r.sleep();
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node {std::make_shared<Node>()};
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}