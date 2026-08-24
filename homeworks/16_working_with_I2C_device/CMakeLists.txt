cmake_minimum_required(VERSION 3.20)
project(working_with_I2C_device CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(working_with_I2C_device src/main.cpp src/i2c/I2CBus.cpp src/sensors/Mpu6050.cpp)

target_include_directories(working_with_I2C_device PRIVATE include)
target_compile_options(working_with_I2C_device PRIVATE -Wall -Wextra -pedantic)
