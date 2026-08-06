cmake_minimum_required(VERSION 3.20)
project(web_protocols CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(
    web_protocols
    src/main.cpp
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

target_include_directories(web_protocols PRIVATE include)
target_compile_options(web_protocols PRIVATE -Wall -Wextra -pedantic)
