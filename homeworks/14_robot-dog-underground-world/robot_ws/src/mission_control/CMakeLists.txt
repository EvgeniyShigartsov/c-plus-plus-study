cmake_minimum_required(VERSION 3.20)
project(mission_control)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(mission_explorer_node src/mission_explorer_node.cpp)
target_include_directories(mission_explorer_node PRIVATE include)

ament_auto_add_executable(payload_action_node src/payload_action_node.cpp)
target_include_directories(payload_action_node PRIVATE include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(explorer_test test/explorer_test.cpp)
    target_include_directories(explorer_test PRIVATE include)
endif()

ament_auto_package()
