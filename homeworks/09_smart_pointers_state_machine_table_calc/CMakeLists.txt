cmake_minimum_required(VERSION 3.20)
project(smart_pointers_state_machine_table_calc CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(
    smart_pointers_state_machine_table_calc
    src/main.cpp
    src/config/ComponentFactory.cpp
    src/config/FileConfigLoader.cpp
    src/providers/JsonTargetProvider.cpp
    src/solvers/AnalyticalSolver.cpp
    src/utils/MathUtils.cpp
    src/states/StateAccelerating.cpp
    src/states/StateDecelerating.cpp
    src/states/StateMoving.cpp
    src/states/StateStopped.cpp
    src/states/StateTurning.cpp
    src/solvers/TableSolver.cpp
    src/BallisticTable.cpp
    src/MissionProcessor.cpp)

target_include_directories(smart_pointers_state_machine_table_calc PRIVATE include)
target_compile_options(smart_pointers_state_machine_table_calc PRIVATE -Wall -Wextra -pedantic)
