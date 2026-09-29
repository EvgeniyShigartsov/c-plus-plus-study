#!/usr/bin/env bash
# Запуск на Raspberry Pi, ЧАСТИНА 1 з 2: serial_bridge + autopilot, дрон - ESP32 по UART (/dev/serial0).
# Оператор запускається окремо на іншій машині (run_hardware_operator.sh) ПІСЛЯ рядка "READY" у виводі цього скрипта.
# Годинник місії дрона стоїть, доки не прийде перше повідомлення від оператора.
# ESP32 має бути прошита з DRONE_CONFIG=<номер-тесту> (idf.py build -DDRONE_CONFIG=N).
# Міст (--reset-drone) на початку сам перезавантажує ESP32, щоб годинник місії йшов з нуля.
# Використання: ./run_hardware_pi.sh <номер-тесту> <ip-оператора> 
#   номер-тесту   - який тесткейс з testing_data використати, 1=01_sample_circles, 2=02_eliptic_trajectories, etc.
#   ip-оператора  - куди слати телеметрію та статус: числовий IP машини з оператором, який бачить Pi

set -u
cd "$(dirname "$0")"

BIN="build"

if [ $# -lt 2 ]; then
  echo "Використання: $0 <номер-тесту> <ip-оператора>"
  echo "Приклад:      $0 4 192.168.2.1"
  exit 1
fi

TEST_N="$1"
OPERATOR_HOST="$2"
TIME_SCALE=1  # на залізі завжди реальний час, але без аргументу автопілот візьме time-scale з config.json, тому передача явна

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
AMMO="data/ammo.json"
BALLISTIC="data/ballistic_table.txt"
SERIAL_DEVICE="/dev/serial0"
DRONE_REBOOT_WAIT_SEC=3  # скільки чекати, поки ESP32 перезавантажиться після команди

echo "тест:      $TEST_N ($DIR)"
echo "хост оператора:  $OPERATOR_HOST"
echo "ESP32 має бути прошита з DRONE_CONFIG=$TEST_N (перезавантажується serial_bridge автоматично)"
echo

PIDS=()

"$BIN/hm20_serial_bridge" --serial-device "$SERIAL_DEVICE" --gcs-host "$OPERATOR_HOST" --reset-drone &
PIDS+=("$!")
sleep "$DRONE_REBOOT_WAIT_SEC"  # ESP32 перезавантажується, її старі дані летять у нікуди, автопілот і оператор ще не запущені
"$BIN/hm20_autopilot" --config-path "$CONFIG" --ammo-path "$AMMO" --ballistic-table "$BALLISTIC" --gcs-host "$OPERATOR_HOST" --time-scale "$TIME_SCALE" &
PIDS+=("$!")

trap 'kill "${PIDS[@]}" 2>/dev/null' INT TERM

sleep 0.5
echo
echo "READY: дрон перезавантажено, автопілот запущено. Можна запускати скрипт оператора (run_hardware_operator.sh)."
echo

wait
