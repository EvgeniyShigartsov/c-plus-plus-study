#!/usr/bin/env bash
# Збірка UART-автопілота для VM середовища з libgpiod v2.
set -euo pipefail
cd "$(dirname "$0")"

g++ -std=c++20 -O2 -Wall -Wextra -pedantic \
    src/main.cpp \
    src/link/UartLink.cpp \
    src/link/UdpLink.cpp \
    src/link/MavlinkReporter.cpp \
    src/link/PacketMappers.cpp \
    src/link/MissionConfig.cpp \
    src/gpio/GpioSignals.cpp \
    src/providers/UartTargetProvider.cpp \
    src/control/DroneController.cpp \
    src/utils/MathUtils.cpp \
    src/MissionProcessor.cpp \
    src/solvers/TableSolver.cpp \
    src/BallisticTable.cpp \
    src/states/StateAccelerating.cpp \
    src/states/StateDecelerating.cpp \
    src/states/StateMoving.cpp \
    src/states/StateStopped.cpp \
    src/states/StateTurning.cpp \
    -Iinclude \
    -isystem include/third_party/c_library_v2 \
    -lgpiod \
    -o script.out

echo "OK: $(pwd)/script.out"
