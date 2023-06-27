#ifndef _I2CAxisServer_H_
#define _I2CAxisServer_H_
#include <i2c_ros/I2CHandle.h>
#include <axis/srv/hard_stop_axis.hpp>
#include <axis/srv/move_axis.hpp>
#include <axis/srv/set_axis_pos.hpp>
#include <axis/srv/set_axis_state.hpp>
#include <axis/srv/soft_stop_axis.hpp>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <utility>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>

/// @brief 
/// hosts various services to control the stepper motors via the i2c bus
/// the configuration @param stepperFile should be written as follows -
/// frame_name   parent_frame_name   varying_element   slave_address
/// @attention
/// varying_element - 0 for xt, 1 for yt, 2 for zt, 3 for xr, 4 for yr, 5 for zr
/// @warning each frame name should be unique, no duplicates allowed
/// @warning each frame should only get one varying axis. e.g. robot_arm frame will have
/// sub frames robot_arm_x that will move only in x direction and robot arm_xr for getting required
/// roll angle. This to be taken care of in next version of the library.
class I2CAxisServer : public rclcpp::Node
{
public:
    I2CAxisServer(const char* i2cBus, const char* stepperFile);
    void sendAllTransforms();
    void (*setPin)(){nullptr};
    void (*clrPin)(){nullptr};

private:
    I2CHandle i2chandle;
    rclcpp::Service<axis::srv::HardStopAxis>::SharedPtr hardStopAxisSrvr;
    rclcpp::Service<axis::srv::MoveAxis>::SharedPtr moveAxisSrvr;
    rclcpp::Service<axis::srv::SetAxisPos>::SharedPtr setAxisPosSrvr;
    rclcpp::Service<axis::srv::SetAxisState>::SharedPtr setAxisStateSrvr;
    rclcpp::Service<axis::srv::SoftStopAxis>::SharedPtr softStopAxisSrvr;
    std::ifstream axisFile;

    /// @brief stores the slave address for given axis
    std::unordered_map<std::string, int> axisMap;

    /// @brief stores what parameter from transform is varying i.e. xt, yt, zt, pitch, roll or yaw for given motor
    /// which is uniquely identified by the slave address
    std::unordered_map<std::string, int> varyingMap;

    /// @brief stores transform for given axis with header set to proper values in constructor
    std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> tfs;

    std::unique_ptr<tf2_ros::TransformBroadcaster> br;

    /// @brief queue of stopped axis at any given moment.
    /// The first axis is the one that was stopped the earliest.
    std::queue<std::string> stopped;

    bool hardStopAxisCb(const axis::srv::HardStopAxis::Request::SharedPtr req, axis::srv::HardStopAxis::Response::SharedPtr res);
    bool moveAxisCb(const axis::srv::MoveAxis::Request::SharedPtr req, axis::srv::MoveAxis::Response::SharedPtr res);
    bool setAxisPosCb(const axis::srv::SetAxisPos::Request::SharedPtr req, axis::srv::SetAxisPos::Response::SharedPtr res);
    bool setAxisStateCb(const axis::srv::SetAxisState::Request::SharedPtr req, axis::srv::SetAxisState::Response::SharedPtr res);
    bool softStopAxisCb(const axis::srv::SoftStopAxis::Request::SharedPtr req, axis::srv::SoftStopAxis::Response::SharedPtr res);
    bool updateTf(const std::string& axisName);
};

#endif