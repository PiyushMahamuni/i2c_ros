#include <i2c_ros/I2CAxisServer.h>

I2CAxisServer::I2CAxisServer(const char *i2c_bus, const char *axisFile) : rclcpp::Node{"i2c_axis_server"},
                                                                          i2chandle{i2c_bus},
                                                                          axisFile{axisFile}
{
    std::string axisName;
    std::string parentAxis;
    int slave;
    int varying;
    RCLCPP_INFO(get_logger(), "Reading from %s", axisFile);
    while (this->axisFile >> axisName >> parentAxis >> varying >> slave)
    {
        axisMap[axisName] = slave;
        geometry_msgs::msg::TransformStamped tf;
        tf.child_frame_id = axisName;
        tf.header.frame_id = parentAxis;
        tfs[axisName] = tf;
        varyingMap[axisName] = varying;
        RCLCPP_INFO(get_logger(), "parsed - %s %s %d %d", axisName.c_str(), parentAxis.c_str(), varying, slave);
    }
    hardStopAxisSrvr = create_service<axis::srv::HardStopAxis>("hard_stop_axis", std::bind(&I2CAxisServer::hardStopAxisCb, this, std::placeholders::_1, std::placeholders::_2));
    moveAxisSrvr = create_service<axis::srv::MoveAxis>("move_axis", std::bind(&I2CAxisServer::moveAxisCb, this, std::placeholders::_1, std::placeholders::_2));
    setAxisPosSrvr = create_service<axis::srv::SetAxisPos>("set_axis_pos", std::bind(&I2CAxisServer::setAxisPosCb, this, std::placeholders::_1, std::placeholders::_2));
    setAxisStateSrvr = create_service<axis::srv::SetAxisState>("set_axis_state", std::bind(&I2CAxisServer::setAxisStateCb, this, std::placeholders::_1, std::placeholders::_2));
    softStopAxisSrvr = create_service<axis::srv::SoftStopAxis>("soft_stop_axis", std::bind(&I2CAxisServer::softStopAxisCb, this, std::placeholders::_1, std::placeholders::_2));
    br = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(get_logger(), "Setup Complete! All axis services online!");
}

bool I2CAxisServer::hardStopAxisCb(const axis::srv::HardStopAxis::Request::SharedPtr req, axis::srv::HardStopAxis::Response::SharedPtr res)
{
    RCLCPP_INFO(get_logger(), "hard_stop_axis called with: axis = %s", req->axis_name.data());
    uint8_t cmd;
    cmd = 2;
    auto it = axisMap.find(req->axis_name);
    if (it == axisMap.end())
    {
        res->success = false;
        RCLCPP_ERROR(get_logger(), "Couldn't find %s in axisMap", req->axis_name.data());
        return true;
    }

    // send hardstop command over the i2c
    I2CWritePacket packet{it->second, cmd};
    i2chandle.send(packet);
    res->success = packet.isSuccess(); // returns if the packet is sent successfully or not
    if (!res->success)
    {
        // return if the command was not sent succesfully
        // this also means that hardstop didn't execute and there's no need to update the transform
        RCLCPP_ERROR(get_logger(), "hard stop command couldn't be sent to slave: %d", it->second);
        return true;
    }
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "hard stop command executed successfully");
// #endif
    // we have lost the certainty of where given frame is after calling hardstop, so update the transform
    float pos;
    updateTf(req->axis_name, pos);
    return true;
}

bool I2CAxisServer::moveAxisCb(const axis::srv::MoveAxis::Request::SharedPtr req, axis::srv::MoveAxis::Response::SharedPtr res)
{
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "move_axis called with: axis = %s, vmax = %f, acc = %f, s = %f", req->axis_name.data(), req->vmax, req->acc, req->s);
// #endif
    const ssize_t size{5};
    uint8_t buff[size];
    auto it = axisMap.find(req->axis_name);
    if (it == axisMap.end())
    {
        res->success = false;
        RCLCPP_ERROR(get_logger(), "couldn't find %s in axisMap", req->axis_name.data());
        return true;
    }
    int &slave{it->second};
    // Send set Vmax in m/s or rad/s command over the i2c bus
    *buff = 12;
    float Vmax{req->vmax};
    memcpy(buff + 1, &Vmax, 4);
    I2CWritePacket packet{slave, buff, size};
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Setting Vmax  to %f (m/s or rad/s) via i2c bus", Vmax);
// #endif
    i2chandle.send(packet);
    if (!packet.isSuccess())
    {
        res->success = false;
        RCLCPP_ERROR(get_logger(), "Failed to set Vmax over the i2c bus for slave: %d", slave);
        return true;
    }
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Vmax set successfully!");
// #endif
    // Send set acceleration in m/s^2 or rad/s^2 command over the i2c bus
    *buff = 10;
    float acc{std::sqrt(req->acc)};
    memcpy(buff + 1, &acc, 4);
    I2CWritePacket packet2{slave, buff, size};
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Setting acc to %f (m/s2 or rad/s2) via i2c bus", acc);
// #endif
    i2chandle.send(packet2);
    if (!packet2.isSuccess())
    {
        res->success = false;
        RCLCPP_ERROR(get_logger(), "Failed to set acc over the i2c bus for slave: %d", slave);
        return true;
    }
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "acc set successfully!");
// #endif
    // Send set relative target in meter or radians command over the i2c bus
    *buff = 9;
    float rt{req->s};
    memcpy(buff + 1, &rt, 4);
    I2CWritePacket packet3{slave, buff, size};
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Setting s to %f (m or rad) via i2c bus", rt);
// #endif
    i2chandle.send(packet3);
    if (!packet3.isSuccess())
    {
        res->success = false;
        RCLCPP_ERROR(get_logger(), "Failed to set s over the i2c bus for slave: %d", slave);
        return true;
    }
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "s set successfully!");
// #endif
    // Send start running command over the i2c bus
    *buff = 5;
    I2CWritePacket packet4{slave, *buff};
    // clear the state pin if a function is given for it
    if (clrPin)
        clrPin();
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Sending start running command over the i2c bus");
// #endif
    i2chandle.send(packet4);
    if (!packet4.isSuccess())
    {
        res->success = false;
        RCLCPP_ERROR(get_logger(), "Failed to send start running command over the i2c bus for slave: %d", slave);
        return true;
    }
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "start running command send successfully!");
// #endif
    // set the pin back if the function is given for it
    if (setPin)
        setPin();
    res->success = true;
    return true;
}

bool I2CAxisServer::setAxisPosCb(const axis::srv::SetAxisPos::Request::SharedPtr req, axis::srv::SetAxisPos::Response::SharedPtr res)
{
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "set_axis_pos called with: axis = %s, pos = %f", req->axis_name.data(), req->pos);
// #endif
    const ssize_t size{5};
    uint8_t buff[size];
    *buff = 11; // command number
    auto it = axisMap.find(req->axis_name);
    res->success = true;
    if (it == axisMap.end())
    {
        res->success = false;
        RCLCPP_ERROR(get_logger(), "[I2CStepperServer] couldn't find %s in stepperMap", req->axis_name.data());
        return true;
    }
    int &slave{it->second};
    float pos{req->pos};
    // in meter or radians
    memcpy(buff + 1, &pos, 4);
    I2CWritePacket packet{slave, buff, size};
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Setting new positoin over the i2c bus");
// #endif
    i2chandle.send(packet);
    res->success = packet.isSuccess();
    if (!res->success)
    {
        RCLCPP_ERROR(get_logger(), "Failed to set new position for slave: %d", slave);
        return true;
    }
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Position set successfully!");
// #endif
    if (!updateTf(req->axis_name, pos))
    {
        RCLCPP_ERROR(get_logger(), "Failed to update position for slave: %d", slave);
        res->success = false;
        return true;
    }
    res->pos = pos;
    return true;
}

bool I2CAxisServer::setAxisStateCb(const axis::srv::SetAxisState::Request::SharedPtr req, axis::srv::SetAxisState::Response::SharedPtr res)
{
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "set_axis_state called with: axis = %s, state = %d", req->axis_name.data(), req->state);
// #endif
    uint8_t buff;
    buff = req->state ? 4 : 3; // 4 - enable, 3 - disable
    auto it = axisMap.find(req->axis_name);
    if (it == axisMap.end())
    {
        res->success = false;
        RCLCPP_ERROR(get_logger(), "Couldn't find %s in stepperMap", req->axis_name.data());
        return true;
    }
    I2CWritePacket packet{it->second, buff};
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Setting axis state over the i2c bus");
// #endif
    i2chandle.send(packet);
    res->success = packet.isSuccess();
    if (!res->success)
    {
        RCLCPP_ERROR(get_logger(), "Failed to set axis state for the slave: %d", it->second);
    }
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "axis state set successfully over the i2c bus!");
// #endif
    // update tf if the axis was just enabled.
    if (req->state)
    {
        float pos;
        if (!updateTf(req->axis_name, pos))
        {
            RCLCPP_ERROR(get_logger(), "Failed to update pos for slave: %d", it->second);
        }
    }
    return true;
}

bool I2CAxisServer::softStopAxisCb(const axis::srv::SoftStopAxis::Request::SharedPtr req, axis::srv::SoftStopAxis::Response::SharedPtr res)
{
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "soft_stop_axis called with: axis = %s", req->axis_name.data());
// #endif
    uint8_t buff;
    buff = 1;
    auto it = axisMap.find(req->axis_name);
    if (it == axisMap.end())
    {
        res->success = false;
        RCLCPP_ERROR(get_logger(), "Couldn't find %s in stepperMap", req->axis_name.data());
        return true;
    }
    I2CWritePacket packet{it->second, buff};
    if (clrPin)
        clrPin();
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "sending soft stop command over the i2c bus");
// #endif
    i2chandle.send(packet);
    res->success = packet.isSuccess();
    if (!res->success)
    {
        RCLCPP_ERROR(get_logger(), "Failed to send soft stop command for the slave: %d", it->second);
        return true;
    }
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "soft stop command send successfully!");
// #endif
    if (setPin)
        setPin();
    stopped.push(req->axis_name);
    return true;
}

bool I2CAxisServer::updateTf(const std::string &axisName, float &pos)
{
    uint8_t buff[4];
    *buff = 7;
    int &slave{axisMap[axisName]};
    I2CWritePacket packet{slave, *buff};
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Requesting pos from axis: %s", axisName.c_str());
// #endif
    i2chandle.send(packet);
    if (!packet.isSuccess())
    {
        RCLCPP_ERROR(get_logger(), "Request Failed!");
        return false;
    }
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Request has been accepted!");
// #endif
    I2CReadPacket rpacket{slave, pos};
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Retrieving pos from axis: %x", axisName.c_str());
// #endif
    i2chandle.receive(rpacket); // updates pos
    if (!rpacket.isSuccess())
    {
        RCLCPP_ERROR(get_logger(), "Failed to retrieve pos!");
        return false;
    }
// #ifdef DEBUG
    RCLCPP_INFO(get_logger(), "Position retrieved successfully! pos = %f", pos);
// #endif
    tf2::Quaternion q;
    auto it{varyingMap.find(axisName)};
    if (it == varyingMap.end())
    {
        return false;
    }
    auto &tf{tfs[axisName]};
    switch (it->second)
    {
    case 0: // Xt
        tf.transform.translation.x = pos;
        break;
    case 1:
        tf.transform.translation.y = pos;
        break;
    case 2:
        tf.transform.translation.z = pos;
        break;
    case 3:
        q.setRPY(pos, 0, 0);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        break;
    case 4:
        q.setRPY(0, pos, 0);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        break;
    case 5:
        q.setRPY(0, 0, pos);
        tf.transform.rotation.x = q.x();
        tf.transform.rotation.y = q.y();
        tf.transform.rotation.z = q.z();
        tf.transform.rotation.w = q.w();
        break; 
    }
    tf.header.stamp = get_clock()->now();
    return true;
}

void I2CAxisServer::sendAllTransforms()
{
    // Update tfs of soft stopped axes
    // iterate over all the elements of stopped
    RCLCPP_INFO(get_logger(), "sendAllTransform is called"); // debug
    // while (!stopped.empty())
    // {
    //     std::string axis{stopped.front()};
    //     stopped.pop();
    //     float pos;
    //     updateTf(axis, pos);
    // }
    for (const auto &tf : tfs)
    {
        RCLCPP_INFO(get_logger(), "Sending a transform"); // debug
        float pos;
        updateTf(tf.first, pos);
        br->sendTransform(tf.second);
    }
}