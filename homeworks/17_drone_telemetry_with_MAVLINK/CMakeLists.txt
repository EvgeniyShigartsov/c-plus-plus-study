cmake_minimum_required(VERSION 3.20)
project(drone_telemetry_with_MAVLINK CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
    drone_telemetry_with_MAVLINK STATIC
    src/main.cpp
    src/link/UartLink.cpp
    src/link/UdpLink.cpp
    src/link/MavlinkReporter.cpp
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
target_compile_options(drone_telemetry_with_MAVLINK PRIVATE -Wall -Wextra -pedantic)

set(MAVLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/third_party/c_library_v2)
if(EXISTS ${MAVLINK_DIR}/common/mavlink.h)
    target_include_directories(drone_telemetry_with_MAVLINK SYSTEM PRIVATE ${MAVLINK_DIR})
else()
    message(
        WARNING
            "MAVLink c_library_v2 not found at ${MAVLINK_DIR} - clone it (git clone --depth 1 https://github.com/mavlink/c_library_v2.git)"
    )
endif()
