#!/usr/bin/env bash
# Збірка UART-автопілота для VM середовища з libgpiod v2.
set -euo pipefail
cd "$(dirname "$0")"

g++ -std=c++20 -O2 -Wall -Wextra -pedantic \
    src/main_uart.cpp \
    src/link/UartLink.cpp \
    src/gpio/GpioSignals.cpp \
    src/providers/UartTargetProvider.cpp \
    src/control/DroneController.cpp \
    src/utils/MathUtils.cpp \
    src/MissionProcessor.cpp \
    src/DronePhysics.cpp \
    src/solvers/TableSolver.cpp \
    src/BallisticTable.cpp \
    src/states/StateAccelerating.cpp \
    src/states/StateDecelerating.cpp \
    src/states/StateMoving.cpp \
    src/states/StateStopped.cpp \
    src/states/StateTurning.cpp \
    -Iinclude \
    -lgpiod \
    -o script.out

echo "OK: $(pwd)/script.out"
