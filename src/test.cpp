#include <i2c_ros/I2CHandle.h>
#include <iostream>
#include <string>

int main()
{
    I2CHandle i2chandle{"/dev/i2c-1"};
    float f;
    std::cout << "Enter slave address: ";
    int slave;
    std::cin >> slave;
    std::cout << std::hex << "Entered slave: " << slave << '\n' << std::dec
              << "Enter a float to echo, any other character to quit\n";
    std::string temp;
    std::getline(std::cin, temp);
    while(std::cin >> f)
    {
        I2CWritePacket wp{slave, f};
        i2chandle.send(wp);
        float echo;
        I2CReadPacket rp{slave, echo};
        i2chandle.receive(rp);
        std::cout << "echo: " << echo << '\n';
    }
    return 0;
}