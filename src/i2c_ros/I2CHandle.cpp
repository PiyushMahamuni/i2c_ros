#include <i2c_ros/I2CHandle.h>

I2CPacket::I2CPacket(int slave, uint8_t *const buff, ssize_t size) : slave{slave}, buff{buff}, size{size}
{
}

const bool& I2CPacket::isProcessed()
{
    return processed;
}

const bool& I2CPacket::isSuccess()
{
    return success;
}

I2CReadPacket::I2CReadPacket(int slave, uint8_t *const buff, ssize_t size) : I2CPacket{slave, buff, size}
{
}

template <typename T>
I2CReadPacket::I2CReadPacket(int slave, T& data): I2CPacket{slave, (uint8_t* const)&data, sizeof(T)}
{
}

template <>
I2CReadPacket::I2CReadPacket<std::string>(int slave, std::string& data) : I2CPacket{slave, (uint8_t* const)data.c_str(), (ssize_t)data.length()}
{
}

template I2CReadPacket::I2CReadPacket<float>(int slave, float& data);
template I2CReadPacket::I2CReadPacket<char>(int slave, char& data);
template I2CReadPacket::I2CReadPacket<int>(int slave, int& data);
template I2CReadPacket::I2CReadPacket<double>(int slave, double& data);
template I2CReadPacket::I2CReadPacket<long>(int slave, long& data);
template I2CReadPacket::I2CReadPacket<unsigned long>(int slave, unsigned long& data);
template I2CReadPacket::I2CReadPacket<unsigned int>(int slave, unsigned int& data);
template I2CReadPacket::I2CReadPacket<uint8_t>(int slave, uint8_t& data);

I2CWritePacket::I2CWritePacket(int slave, const uint8_t *const buff, ssize_t size) : I2CPacket{slave, const_cast<uint8_t* const>(buff), size}
{
}

template <typename T>
I2CWritePacket::I2CWritePacket(int slave, const T& data) : I2CPacket{slave, (uint8_t* const)&data, sizeof(T)}
{
}

template <>
I2CWritePacket::I2CWritePacket<std::string>(int slave, const std::string& data): I2CPacket{slave, (uint8_t* const)data.c_str(), (ssize_t)data.length()}
{
}

template I2CWritePacket::I2CWritePacket<float>(int slave, const float& data);
template I2CWritePacket::I2CWritePacket<int>(int slave, const int& data);
template I2CWritePacket::I2CWritePacket<double>(int slave, const double& data);
template I2CWritePacket::I2CWritePacket<long>(int slave, const long& data);
template I2CWritePacket::I2CWritePacket<unsigned long>(int slave, const unsigned long& data);
template I2CWritePacket::I2CWritePacket<unsigned int>(int slave, const unsigned int& data);
template I2CWritePacket::I2CWritePacket<uint8_t>(int slave, const uint8_t& data);

I2CHandle::I2CHandle(const char *bus) : bus{bus}, fd{open(bus, O_RDWR)}
{
    // get resource
    if(fd < 0)
    {
        std::cout << "[I2CHandle] Failed to open " << bus << " bus\n";
        throw FailedToOpenBusException{bus};
    }
    std::cout << "[I2CHandle] " << bus << " bus opened successfully\n";
}

bool I2CHandle::send(I2CWritePacket& packet)
{
    packet.processed = packet.success = true;
    if(fd < 0)
    {
        std::cout << "[I2CHandle] Failed to open bus " << bus << ", can't perform requested write operation\n";
        return packet.success = false;
        // throw FailedToOpenBusException{bus};
    }
    I2CMutex.lock();
    // connect to slave
    if(ioctl(fd, I2C_SLAVE, packet.slave) < 0)
    {
        std::cout << "[I2CHandle] Failed to connect to device: " << packet.slave << '\n';
        I2CMutex.unlock();
        return packet.success = false;
        // throw FailedToConnectException{packet.slave};
    }
    if(write(fd, packet.buff, packet.size) != packet.size)
    {
        std::cout << "[I2CHandle] Failed to write to device: " << packet.slave << '\n';
        I2CMutex.unlock();
        return packet.success = false;
        // throw FailedToWriteException{packet.slave};
    }
    I2CMutex.unlock();
    return true;
}

bool I2CHandle::receive(I2CReadPacket& packet)
{
    packet.processed = packet.success = true;
    if(fd < 0)
    {
        std::cout << "[I2CHandle] Failed to open bus " << bus << ", can't perform requested read operation\n";
        return packet.success = false;
        // throw FailedToOpenBusException{bus};
    }
    I2CMutex.lock();
    if(ioctl(fd, I2C_SLAVE, packet.slave) < 0)
    {
        std::cout << "[I2CHandle] Failed to connect to device: " << packet.slave << '\n';
        I2CMutex.unlock();
        return packet.success = false;
        // throw FailedToConnectException{packet.slave};
    }
    if(read(fd, packet.buff, packet.size) != packet.size)
    {
        std::cout << "[I2CHandle] Failed to read from device: " << packet.slave << '\n';
        I2CMutex.unlock();
        return packet.success = false;
        // throw FailedToReadException{packet.slave};
    }
    I2CMutex.unlock();
    return true;
}

// Destructors
I2CHandle::~I2CHandle()
{
}

I2CPacket::~I2CPacket()
{
}

I2CWritePacket::~I2CWritePacket()
{
}

I2CReadPacket::~I2CReadPacket()
{
}

// ----------------------------------------Exceptions---------------------------------------- 

const char *OutOfBoundException::what()
{
    return "index is out of bounds\n";
}

FailedToConnectException::FailedToConnectException(int slave): slave{slave}
{
}

const char *FailedToConnectException::what()
{
    std::string msg{"Failed to connect to slave:  "};
    msg.back() = slave + '0';
    return msg.c_str();
}

FailedToWriteException::FailedToWriteException(int slave) : slave{slave}
{
}

const char *FailedToWriteException::what()
{
    std::string msg{"Failed to write to slave:  "};
    msg.back() = slave + '0';
    return msg.c_str();
}

FailedToReadException::FailedToReadException(int slave) : slave{slave}
{
}

const char *FailedToReadException::what()
{
    std::string msg{"Failed to read from slave:  "};
    msg.back() = slave + '0';
    return msg.c_str();
}

FailedToOpenBusException::FailedToOpenBusException(const char *bus) : bus{bus}
{
}

const char *FailedToOpenBusException::what()
{
    std::string msg{"Failed to open "};
    msg += bus;
    msg += " bus";
    return msg.c_str();
}