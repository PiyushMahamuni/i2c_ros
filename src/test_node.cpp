#include <i2c_ros/I2CHandle.h>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <string>
#include <thread>

class TestI2CHandle : public rclcpp::Node
{
public:
    TestI2CHandle(int slave) : rclcpp::Node{"test_i2c_handle"}, slave{slave}, i2chandle{"/dev/i2c-1"}
    {
        RCLCPP_INFO(get_logger(), " Setup Complete!");
        th = std::thread(std::bind(&TestI2CHandle::test, this));
    }

private:
    int slave;
    I2CHandle i2chandle;
    std::thread th;

    void test()
    {
        std::cout << "Enter a float to echo, any other character to quit\n";
        float f;
        while(std::cin >> f)
        {
            I2CWritePacket wp{slave, f};
            i2chandle.send(wp);
            float echo;
            I2CReadPacket rp{slave, echo};
            i2chandle.receive(rp);
            std::cout << "echo: " << echo << '\n';
        }
        rclcpp::shutdown();
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    std::cout << "Enter slave address: ";
    int slave;
    std::cin >> slave;
    auto node {std::make_shared<TestI2CHandle>(slave)};
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}