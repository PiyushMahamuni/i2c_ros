cmake_minimum_required(VERSION 3.5)
project(i2c_ros)

# Set C++ standard to 14
set(CMAKE_CXX_STANDARD 14)

# Enable compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(axis REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Create libraries
add_library(I2CHandle
  src/i2c_ros/src/I2CHandle.cpp
)
add_library(I2CAxisServer
  src/i2c_ros/src/I2CAxisServer.cpp
)

# Link libraries
ament_target_dependencies(I2CAxisServer
  rclcpp
  axis
  geometry_msgs
)

# Install libraries
install(TARGETS
  I2CHandle
  I2CAxisServer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# Create executables
add_executable(testi2c_node src/test_node.cpp)
add_executable(testi2c src/test.cpp)
add_executable(testserver src/test_server.cpp)

# Link executables
ament_target_dependencies(testi2c_node rclcpp axis)
ament_target_dependencies(testserver rclcpp axis tf2_ros)

target_link_libraries(testi2c_node I2CHandle)
target_link_libraries(testi2c I2CHandle)
target_link_libraries(testserver I2CAxisServer I2CHandle)

# Install executables
install(TARGETS
  testi2c_node
  testi2c
  testserver
  DESTINATION lib/${PROJECT_NAME}
)

# Create ament package
ament_package()
