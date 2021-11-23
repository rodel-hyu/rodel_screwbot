cmake_minimum_required(VERSION 3.5)
project(rodel_screwbot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include
  src/Adafruit_BNO055
  src/CANOpen)

# RF Node
add_executable(rf_node
  src/rf_node.cpp)
ament_target_dependencies(rf_node rclcpp sensor_msgs)

# IMU Node
add_executable(imu_node
  src/imu_node.cpp
  src/Adafruit_BNO055/Adafruit_BNO055.cpp
  src/Adafruit_BNO055/Adafruit_I2CDevice.cpp
  src/Adafruit_BNO055/Adafruit_Sensor.cpp)
ament_target_dependencies(imu_node rclcpp sensor_msgs)

# DISPLAY Node
add_executable(display_node
  src/display_node.cpp)
ament_target_dependencies(display_node rclcpp std_msgs sensor_msgs)

# MAXON Node
add_executable(maxon_node
  src/maxon_node.cpp
  src/CANOpen/CANOpen.c
  src/CANOpen/CANOpen_hw_appl.c)
ament_target_dependencies(maxon_node rclcpp std_msgs sensor_msgs)

# MD200T Node
add_executable(md200t_node
  src/md200t_node.cpp
)
ament_target_dependencies(md200t_node rclcpp std_msgs sensor_msgs)

install(TARGETS
  rf_node
  imu_node
  display_node
  maxon_node
  md200t_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
