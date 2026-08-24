#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

g++ -std=c++20 -O2 -Wall -Wextra -pedantic \
    src/main.cpp \
    src/i2c/I2CBus.cpp \
    src/sensors/Mpu6050.cpp \
    -Iinclude \
    -o script.out

echo "OK: $(pwd)/script.out"
