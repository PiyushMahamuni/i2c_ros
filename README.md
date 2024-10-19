ROS package with I2CHandle C++ header to make it easier for any SBCs running linux to talk to any other i2c device as master.
The I2CHandle library is not ROS specific however it is wrapped inside a ROS package.
There's another library I2CAxisServer which is first half of the other library I2CAxisController that it works with to control the axis (position, motion) of any robot.
