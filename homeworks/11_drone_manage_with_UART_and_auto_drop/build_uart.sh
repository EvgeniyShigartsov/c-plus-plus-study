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
    -Iinclude \
    -lgpiod \
    -o hm11_uart_autopilot

echo "OK: $(pwd)/hm11_uart_autopilot"
