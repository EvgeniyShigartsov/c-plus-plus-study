cmake_minimum_required(VERSION 3.20)
project(drone_telemetry_with_MAVLINK CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ДЗ 17 збирається тільки локально у емуляторі Linux, на MacOS це Lima. Для цього треба вручну
# клонувати  MAVLink c_library_v2
set(MAVLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/third_party/c_library_v2)
if(NOT EXISTS ${MAVLINK_DIR}/common/mavlink.h)
    message(STATUS "MAVLink c_library_v2 not found at ${MAVLINK_DIR} - skipping homeworks/17.")
    return()
endif()

add_library(
    drone_telemetry_with_MAVLINK STATIC
    src/main.cpp
    src/link/UartLink.cpp
    src/link/UdpLink.cpp
    src/link/MavlinkUtil.cpp
    src/link/MavlinkReporter.cpp
    src/link/MavlinkDrop.cpp
    src/link/PacketMappers.cpp
    src/link/MissionConfig.cpp
    src/gpio/GpioSignals.cpp
    src/providers/UartTargetProvider.cpp
    src/control/DroneController.cpp
    src/utils/MathUtils.cpp
    src/MissionProcessor.cpp
    src/solvers/TableSolver.cpp
    src/BallisticTable.cpp
    src/states/StateAccelerating.cpp
    src/states/StateDecelerating.cpp
    src/states/StateMoving.cpp
    src/states/StateStopped.cpp
    src/states/StateTurning.cpp)

target_include_directories(drone_telemetry_with_MAVLINK PRIVATE include)
target_include_directories(drone_telemetry_with_MAVLINK SYSTEM PRIVATE ${MAVLINK_DIR})
target_compile_options(drone_telemetry_with_MAVLINK PRIVATE -Wall -Wextra -pedantic)
