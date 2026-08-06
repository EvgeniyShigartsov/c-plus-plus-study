cmake_minimum_required(VERSION 3.20)
project(drone_manage_with_UART_and_auto_drop CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
    hm15_web_protocols STATIC
    src/main.cpp
    src/link/UartLink.cpp
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

target_include_directories(hm15_web_protocols PRIVATE include)
target_compile_options(hm15_web_protocols PRIVATE -Wall -Wextra -pedantic)
