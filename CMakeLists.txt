cmake_minimum_required(VERSION 3.16)
project(epmc_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

# Find libi2c (make sure libi2c-dev is installed)
find_library(I2C_LIB i2c REQUIRED)

add_executable(motor_control examples/motor_control.cpp)
target_link_libraries(motor_control PRIVATE ${I2C_LIB})