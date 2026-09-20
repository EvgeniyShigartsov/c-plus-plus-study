#!/usr/bin/env bash
# Запуск на Raspberry Pi: serial_bridge + autopilot + operator_sim, дрон - ESP32 по UART (/dev/serial0).
# ESP32 має бути прошита з DRONE_CONFIG=<номер-тесту> (idf.py build -DDRONE_CONFIG=N) і ТІЛЬКИ ЩО перезавантажена (RESET):
# годинник дрона йде від його старту, а оператор бере з нього час сценарію і положення цілей.
# Використання: ./run_hardware.sh [сценарій-оператора] [номер-тесту] [time-scale]
#   сценарій-оператора - ім'я файлу в data/scenarios/ без розширення .txt. Дефолт: 01_clean_attack
#   номер-тесту        - який тесткейс з testing_data використати, 1=01_sample_circles, 2=02_eliptic_trajectories, etc. Дефолт: 1
#   time-scale         - на залізі 1 (реальний час). Дефолт: 1

set -u
cd "$(dirname "$0")"

BIN="build"

SCENARIO="${1:-01_clean_attack}"
TEST_N="${2:-1}"
TIME_SCALE="${3:-1}"

case "$TEST_N" in
  1)  DIR="01_sample_circles" ;;
  2)  DIR="02_eliptic_trajectories" ;;
  3)  DIR="03_visimky_lissaju_1-2" ;;
  4)  DIR="04_star_trajectories" ;;
  5)  DIR="05_lissaju_complex_curves" ;;
  6)  DIR="06_fast_drone_slow_targets" ;;
  7)  DIR="07_heavy_ammo" ;;
  8)  DIR="08_gliding_ammo" ;;
  9)  DIR="09_cardioids_eptirohoids" ;;
  10) DIR="10_extreme_far_fast" ;;
  *)  echo "невідомий номер тесту: $TEST_N"; exit 1 ;;
esac

NUM=$(printf '%02d' "$TEST_N")
TEST_DIR="data/testing_data/$DIR"
CONFIG="$TEST_DIR/${NUM}_config.json"
TARGETS="$TEST_DIR/${NUM}_targets.json"
AMMO="data/ammo.json"
BALLISTIC="data/ballistic_table.txt"
OPERATOR_SCENARIO="data/scenarios/$SCENARIO.txt"
SERIAL_DEVICE="/dev/serial0"

echo "сценарій:  $OPERATOR_SCENARIO"
echo "тест:      $TEST_N ($DIR), time-scale $TIME_SCALE"
echo "ESP32 має бути прошита з DRONE_CONFIG=$TEST_N і щойно перезавантажена"
echo

PIDS=()

"$BIN/hm20_serial_bridge" --serial-device "$SERIAL_DEVICE" &
PIDS+=("$!")
"$BIN/hm20_autopilot" --config-path "$CONFIG" --ammo-path "$AMMO" --ballistic-table "$BALLISTIC" --time-scale "$TIME_SCALE" &
PIDS+=("$!")
sleep 0.2
"$BIN/hm20_operator_sim" --operator-scenario "$OPERATOR_SCENARIO" --config-path "$CONFIG" --ammo-path "$AMMO" --targets "$TARGETS" --time-scale "$TIME_SCALE" --sim-output "simulation.json" &
PIDS+=("$!")

trap 'kill "${PIDS[@]}" 2>/dev/null' INT TERM

wait
