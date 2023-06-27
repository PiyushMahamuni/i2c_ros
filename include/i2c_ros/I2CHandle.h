#ifndef _I2CHandle_H_
#define _I2CHandle_H_
#include <linux/i2c-dev.h>
#include <cstdint>
#include <unistd.h>
#include <cstdio>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <exception>
#include <chrono>
#include <thread>
#include <mutex>
#include <iostream>

class OutOfBoundException : public std::exception
{
public:
    virtual const char *what();
};

class FailedToConnectException : public std::exception
{
private:
    int slave;

public:
    FailedToConnectException(int slave);
    virtual const char *what();
};

class FailedToWriteException : public std::exception
{
private:
    int slave;

public:
    FailedToWriteException(int slave);
    virtual const char *what();
};

class FailedToReadException : public std::exception
{
private:
    int slave;

public:
    FailedToReadException(int slave);
    virtual const char *what();
};

class FailedToOpenBusException : public std::exception
{
private:
    const char *bus;
public:
    FailedToOpenBusException(const char *bus);
    virtual const char *what();
};

/// @brief Abstract Class
/// data to be transmitted or received as over the I2C bus as a string of bytes
/// collectively called as I2CPacket
/// \param slave slave address that this packet is supposed to be communicated with i.e.
/// either received or tranmitted to
/// \param buff pointer to uint8_t buffer i.e. bytes. It doesn't allocate buffer itself.
/// \param size size of the buffer
class I2CPacket
{
    friend class I2CHandle;

protected:
    int slave;
    uint8_t *buff{nullptr};
    ssize_t size;
    bool processed{false}; // if the packet has been handled or not
    bool success{false}; // if write/read operation was a success or not
    virtual void execute(int fd) = 0;

public:
    /// @brief 
    /// @param slave address of the slave the intended packet is meant to be sent/received from
    /// @param buff pointer to buffer of uint8_t i.e. bytes for the data to be sent from or received into
    /// @param size size of the buffer to be processed
    I2CPacket(int slave, uint8_t *const buff, ssize_t size);

    /// Returns if the read/write operation was a success
    const bool& isSuccess();

    /// Returns if the Packet has been processed
    const bool& isProcessed();
    virtual ~I2CPacket();
};


/// @brief 
/// I2CPacket to be read from the given slave address to the given buffer
class I2CReadPacket : public I2CPacket
{
    friend class I2CHandle;

private:
    virtual void execute(int fd) final;

public:
    /// @brief 
    /// @param slave address of the I2C slave the packet is meant to be read from
    /// @param buff pointer to buffer of uint8_t i.e. bytes to store received data into
    /// @param size size of the buffer
    I2CReadPacket(int slave, uint8_t *const buff, ssize_t size);
    
    /// @brief 
    /// @tparam T type of the data to be read
    /// @param slave address of the I2C slave to read the data from
    /// @param data reference of type T to read the data into
    template<typename T>
    I2CReadPacket(int slave, T& data);
    virtual ~I2CReadPacket() final;
};


/// @brief 
/// I2CPacket to be transmitted to  the given slave address from the given buffer
class I2CWritePacket : public I2CPacket
{
    friend class I2CHandle;

private:
    virtual void execute(int fd) final;

public:
    /// @brief 
    /// @param slave address of the I2C slave the packet is meant to be sent to
    /// @param buff pointer to buffer of uint8_t i.e. bytes to send the data from
    /// @param size size of the buffer
    I2CWritePacket(int slave, const uint8_t *const buff, ssize_t size);
    template<typename T>
    I2CWritePacket(int slave, const T& data);
    virtual ~I2CWritePacket() final;
};

/// @brief
/// Handles all the read/write operations done on the I2C bus sequentially and synchronously.
/// Note, this class doesn't support multithreading
/// Do not create multiple instances for the same I2C bus!
/// \param bus name of the I2C bus
class I2CHandle
{
private:
    const char* bus;
    int fd{-1}; // file descriptor
    std::mutex I2CMutex;

public:
    I2CHandle(const char *bus);
    ~I2CHandle();

    bool send(I2CWritePacket& packet);
    bool receive(I2CReadPacket& packet);
};
#endif