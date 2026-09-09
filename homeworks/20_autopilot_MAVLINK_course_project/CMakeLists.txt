cmake_minimum_required(VERSION 3.20)
project(autopilot_mavlink CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ДЗ 20 збирається тільки локально у емуляторі Linux, на MacOS це Lima. Для цього треба вручну
# клонувати  MAVLink c_library_v2
set(MAVLINK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/third_party/c_library_v2)
if(NOT EXISTS ${MAVLINK_DIR}/common/mavlink.h)
    message(STATUS "MAVLink c_library_v2 not found at ${MAVLINK_DIR} - skipping homeworks/20.")
    return()
endif()

set(HM20_WARNINGS -Wall -Wextra -pedantic)

add_library(
    hm20_guidance STATIC
    src/control/DroneController.cpp
    src/utils/MathUtils.cpp
    src/MissionProcessor.cpp
    src/providers/UartTargetProvider.cpp
    src/solvers/TableSolver.cpp
    src/BallisticTable.cpp
    src/states/StateAccelerating.cpp
    src/states/StateDecelerating.cpp
    src/states/StateMoving.cpp
    src/states/StateStopped.cpp
    src/states/StateTurning.cpp)
target_include_directories(hm20_guidance PUBLIC include)
target_compile_options(hm20_guidance PRIVATE ${HM20_WARNINGS})

add_library(hm20_mavlink STATIC src/link/UdpLink.cpp src/link/MavlinkUtil.cpp
                                src/link/MavlinkReporter.cpp src/link/MavlinkDrop.cpp)
target_include_directories(hm20_mavlink PUBLIC include)
target_include_directories(hm20_mavlink SYSTEM PUBLIC ${MAVLINK_DIR})
target_compile_options(hm20_mavlink PRIVATE ${HM20_WARNINGS})

foreach(APP autopilot vehicle_sim operator_sim verdict)
    add_executable(hm20_${APP} src/apps/${APP}.cpp)
    target_link_libraries(hm20_${APP} PRIVATE hm20_guidance hm20_mavlink)
    target_compile_options(hm20_${APP} PRIVATE ${HM20_WARNINGS})
endforeach()
