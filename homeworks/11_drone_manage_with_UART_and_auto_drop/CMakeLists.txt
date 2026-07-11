cmake_minimum_required(VERSION 3.20)
project(drone_manage_with_UART_and_auto_drop CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(
    drone_manage_with_UART_and_auto_drop
    src/main_test_suit_arg_as_number.cpp
    src/config/ComponentFactory.cpp
    src/config/FileConfigLoader.cpp
    src/providers/ThreadSafeTargetProvider.cpp
    src/solvers/AnalyticalSolver.cpp
    src/utils/MathUtils.cpp
    src/states/StateAccelerating.cpp
    src/states/StateDecelerating.cpp
    src/states/StateMoving.cpp
    src/states/StateStopped.cpp
    src/states/StateTurning.cpp
    src/solvers/TableSolver.cpp
    src/BallisticTable.cpp
    src/DronePhysics.cpp
    src/MissionProcessor.cpp)

target_include_directories(drone_manage_with_UART_and_auto_drop PRIVATE include)
target_compile_options(drone_manage_with_UART_and_auto_drop PRIVATE -Wall -Wextra -pedantic)

# Компіляційна перевірка UART-сорсів у девконтейнері, сам бінарник збирає build_uart.sh у VM (у мене
# це Lima), бо потребує libgpiod v2, якого тут немає.
add_library(hm11_uart_compile_check STATIC src/main_uart.cpp src/link/UartLink.cpp)

target_include_directories(hm11_uart_compile_check PRIVATE include)
target_compile_options(hm11_uart_compile_check PRIVATE -Wall -Wextra -pedantic)
